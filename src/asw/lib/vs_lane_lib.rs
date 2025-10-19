use crate::calibration::LaneCalibration;
use opencv::{
    Result,
    core::AlgorithmHint::ALGO_HINT_DEFAULT,
    core::{self, CV_8UC1, CV_8UC3, DECOMP_LU, Mat, Point, Scalar, Size, UMat, Vector},
    highgui, imgproc,
    prelude::*,
    videoio,
};
use rayon::prelude::*;
use std::f64::consts::PI;
use std::time::Instant;

/// 본 코드는 mode 값(CAM_MODE)을 통해 웹캠 혹은 동영상 파일을 선택하여 처리하도록 작성했습니다.
/// 만약 `CAM_MODE == 1`이면 웹캠에서 실시간 영상을 읽고,
/// 그 외의 값이면 사전에 지정한 mp4 파일(`"./video/challenge.mp4"`)을 읽어옵니다.
const CAM_MODE: i32 = 1;

/// 파이프라인 함수들에서 공통으로 사용할 `Result` 타입 별칭입니다.
/// Rust에서는 `Result<T, E>`를 반환할 때 `?` 연산자를 이용해 매우 간편하게
/// 에러 처리를 위임할 수 있습니다. C/C++ 대비해서 매번 if 문으로 에러를 확인하고
/// 되돌려주는 번거로움을 줄여줍니다.
pub type LaneDetectionResult<T> = Result<T>;

/// OpenCV를 이용해 차선 검출을 수행하는 핵심 파이프라인 구조체입니다.
///
/// # 주요 멤버
///
/// - `width`, `height`: 입력(카메라/영상) 해상도
/// - `window_margin`: 슬라이딩 윈도우 좌우 탐색 폭
/// - `vertices`: ROI(관심영역) 설정용 꼭짓점들
/// - `transform_matrix`, `inv_transform_matrix`: 투시/역투시 변환 행렬
/// - `leftx_mid`, `rightx_mid`: 화면 중앙 기준 왼/오 차선의 중간 x좌표
/// - `leftx_base`, `rightx_base`: 최근 슬라이딩 윈도우에서 찾은 차선 기저(base) 좌표
/// - `left_a/b/c`, `right_a/b/c`: 최근 프레임에서 추정한 2차 곡선 계수를 누적 저장
/// - `ploty`: 0~height-1 범위의 y좌표 리스트(차선 곡선 계산에 사용)
/// - `prev_angle`, `steering_angle`: 이전/현재 프레임의 조향각(deg 단위)
/// - `exit_flag`: 메인 루프 종료 플래그
///
/// # Rust에서 구조체 장점
/// - C/C++의 struct와 달리, 필드를 private으로 막고 `impl`을 통해
///   안전한 접근을 제공하기가 편리합니다.
/// - 필드 타입에 대해 컴파일 시점에 철저하게 체크하므로, null 포인터나
///   잘못된 타입 변환으로 인한 런타임 에러가 줄어듭니다.
pub struct Pipeline {
    /// 입력 영상(또는 카메라)의 가로 해상도
    width: i32,
    /// 입력 영상(또는 카메라)의 세로 해상도
    height: i32,

    /// 슬라이딩 윈도우에서 좌우로 탐색할 여유폭 (margin)
    window_margin: i32,

    /// 가우시안 블러 커널 크기 (가로, 세로)
    gaussian_kernel: (i32, i32),
    /// 가우시안 블러 시그마 값 (X, Y)
    gaussian_sigma: (f64, f64),

    /// 캐니 엣지 하한/상한 임계값
    canny_low_threshold: f64,
    canny_high_threshold: f64,
    /// 캐니 엣지 연산 시 커널 크기
    canny_aperture_size: i32,

    /// 모폴로지 연산 커널 크기 및 반복 횟수
    morphology_kernel_size: (i32, i32),
    morphology_iterations: i32,

    /// 슬라이딩 윈도우 탐색 파라미터
    sliding_window_count: i32,
    sliding_window_margin: i32,
    sliding_window_minpix: i32,
    sliding_window_draw_debug: bool,
    lane_points_threshold: usize,
    search_poly_margin: i32,

    /// 관심 영역(ROI)을 정의할 때 사용할 꼭짓점들의 좌표 (4개의 Point)
    vertices: Vec<Point>,

    /// 투시 변환(전방 시야 -> Bird's-eye) 행렬 (3x3)
    transform_matrix: Mat,
    /// 역투시 변환(Bird's-eye -> 원래 시야) 행렬 (3x3)
    inv_transform_matrix: Mat,

    /// 왼쪽 차선 기준점을 잡기 위한 중간 x좌표
    leftx_mid: i32,
    /// 오른쪽 차선 기준점을 잡기 위한 중간 x좌표
    rightx_mid: i32,
    /// 최근 탐색한 왼쪽 차선의 기저(base) x좌표
    leftx_base: i32,
    /// 최근 탐색한 오른쪽 차선의 기저(base) x좌표
    rightx_base: i32,

    /// 왼쪽 차선 2차 곡선 계수(a, b, c) 이력을 저장하는 벡터
    pub(crate) left_a: Vec<f64>,
    pub(crate) left_b: Vec<f64>,
    pub(crate) left_c: Vec<f64>,

    /// 오른쪽 차선 2차 곡선 계수(a, b, c) 이력을 저장하는 벡터
    pub(crate) right_a: Vec<f64>,
    pub(crate) right_b: Vec<f64>,
    pub(crate) right_c: Vec<f64>,

    /// 플롯용 y좌표 목록(0부터 height-1까지)
    ploty: Vec<f64>,

    /// 이전 프레임에서 계산된 조향각 (도 단위)
    prev_angle: f64,

    /// 현재 프레임에서 계산된 조향각 (도 단위)
    steering_angle: f64,

    visible: bool,

    /// 프로그램 종료를 위한 플래그
    pub(crate) exit_flag: bool,

    /// 단순 1차원 칼만 필터 상태 (차선 각 추정값)
    kalman_estimate: f64,
    /// 칼만 필터 공분산
    kalman_covariance: f64,
    /// 칼만 필터 프로세스 노이즈
    kalman_process_noise: f64,
    /// 칼만 필터 측정 노이즈
    kalman_measurement_noise: f64,
    /// 칼만 필터 사용 여부
    use_kalman_filter: bool,
}

impl Pipeline {
    /// 새로운 `Pipeline` 구조체를 생성합니다.
    ///
    /// # 동작
    /// - 해상도(1280x720) 설정
    /// - ROI(관심영역) 꼭짓점 설정
    /// - 투시 변환/역투시 변환 행렬 계산
    /// - 슬라이딩 윈도우 파라미터 지정
    /// - 0~height 범위의 y좌표 미리 저장
    ///
    /// # 반환
    /// - 초기화된 `Pipeline` 객체
    ///
    /// # 에러 처리
    /// - OpenCV 함수(`imgproc::get_perspective_transform`) 호출 시 발생할 수 있는 에러는
    ///   Rust의 `?` 연산자를 통해 호출부로 전파됩니다.
    ///
    /// # C/C++ 대비 Rust 문법 장점
    /// - `Result`와 `?` 연산자를 통해 에러를 핸들링하며, 예외(exception)가 없어
    ///   제어 흐름이 더 명시적이고 예측 가능합니다.
    pub fn new_with_settings(use_kalman_filter: bool) -> Result<Self> {
        let calibration = LaneCalibration::default();
        let camera = calibration.camera;
        let image_calibration = calibration.image;
        let processing = calibration.processing;

        let width = camera.width;
        let height = camera.height;

        let window_margin = processing.sliding.display_margin;

        // ROI(관심영역) 정의를 위한 꼭짓점들(좌하, 좌상, 우상, 우하)
        let vertices = image_calibration.roi.to_points();

        // 투시 변환(원근 -> 직사)용 좌표 설정
        let points_src = image_calibration.perspective.source_points();
        let points_dst = image_calibration.perspective.destination_points();

        // 투시 변환 행렬 (3x3)
        let transform_matrix = imgproc::get_perspective_transform(
            &Mat::from_slice_2d(&[&points_src])?,
            &Mat::from_slice_2d(&[&points_dst])?,
            DECOMP_LU,
        )?;

        // 역투시 변환 행렬 (3x3)
        let inv_transform_matrix = imgproc::get_perspective_transform(
            &Mat::from_slice_2d(&[&points_dst])?,
            &Mat::from_slice_2d(&[&points_src])?,
            DECOMP_LU,
        )?;

        // 좌우 차선 중앙 기준
        let leftx_mid = width / 4;
        let rightx_mid = width * 3 / 4;

        let leftx_base = leftx_mid;
        let rightx_base = rightx_mid;

        // 0부터 height-1까지의 y좌표를 저장
        let ploty: Vec<f64> = (0..height).map(|i| i as f64).collect();

        let enabled_kalman = use_kalman_filter && processing.kalman.enabled;

        Ok(Self {
            width,
            height,
            window_margin,
            gaussian_kernel: processing.filtering.gaussian_kernel,
            gaussian_sigma: processing.filtering.gaussian_sigma,
            canny_low_threshold: processing.filtering.canny_low_threshold,
            canny_high_threshold: processing.filtering.canny_high_threshold,
            canny_aperture_size: processing.filtering.canny_aperture_size,
            morphology_kernel_size: processing.morphology.kernel_size,
            morphology_iterations: processing.morphology.iterations,
            sliding_window_count: processing.sliding.window_count,
            sliding_window_margin: processing.sliding.search_margin,
            sliding_window_minpix: processing.sliding.minpix,
            sliding_window_draw_debug: processing.sliding.draw_debug_windows,
            lane_points_threshold: processing.sliding.required_points,
            search_poly_margin: processing.sliding.search_poly_margin,
            vertices,
            transform_matrix,
            inv_transform_matrix,
            leftx_mid,
            rightx_mid,
            leftx_base,
            rightx_base,
            left_a: vec![0.0],
            left_b: vec![0.0],
            left_c: vec![leftx_mid as f64],
            right_a: vec![0.0],
            right_b: vec![0.0],
            right_c: vec![rightx_mid as f64],
            ploty,
            prev_angle: 0.0,
            steering_angle: 0.0,
            visible: true,
            exit_flag: false,
            kalman_estimate: processing.kalman.initial_estimate,
            kalman_covariance: processing.kalman.initial_covariance,
            kalman_process_noise: processing.kalman.process_noise,
            kalman_measurement_noise: processing.kalman.measurement_noise,
            use_kalman_filter: enabled_kalman,
        })
    }

    /// 기본 설정(슬라이딩 윈도우 + 칼만 필터 비사용)으로 파이프라인을 생성합니다.
    pub fn new() -> Result<Self> {
        Self::new_with_settings(false)
    }

    /// 칼만 필터 사용 여부를 반환합니다.
    pub fn use_kalman(&self) -> bool {
        self.use_kalman_filter
    }

    /// 최근 계산된 조향각(필터 적용 전)을 반환합니다.
    pub fn previous_angle(&self) -> f64 {
        self.prev_angle
    }

    /// 입력 영상을 그레이스케일로 변환합니다.
    ///
    /// # 인자
    /// * `img` - BGR 색상 영상
    ///
    /// # 반환
    /// * 그레이스케일 영상 (`Mat`)
    ///
    /// # 주석
    /// - Rust에서 함수 호출 시 빌린 참조(&)를 사용합니다.
    ///   이는 C++의 레퍼런스와 유사하지만, Rust의 빌림 규칙(Borrow Checker)에 의해
    ///   컴파일 시점에 안전성이 보장됩니다.
    pub(crate) fn gray_scale(&self, img: &Mat) -> Result<UMat> {
        let mut gray = UMat::new_def();
        imgproc::cvt_color(
            img,
            &mut gray,
            imgproc::COLOR_BGR2GRAY,
            0,
            ALGO_HINT_DEFAULT,
        )?;
        Ok(gray)
    }

    /// 히스토그램 평활화 예시 함수(현재 사용 안 함).
    /// OpenCV의 `equalizeHist`를 래핑한 간단한 예시입니다.
    ///
    /// # Rust-C++ 비교
    /// - C++에선 OpenCV 함수를 직접 호출해도 되지만,
    ///   Rust에서는 FFI(외부 함수 인터페이스)를 안전하게 감싼 `opencv` crate를 통해
    ///   에러나 메모리 관리를 보다 견고히 할 수 있습니다.
    #[allow(dead_code)]
    fn equalize_hist(&self, img: &Mat) -> Result<Mat> {
        let mut dst = Mat::default();
        imgproc::equalize_hist(img, &mut dst)?;
        Ok(dst)
    }

    /// 가우시안 블러를 적용하여 영상 노이즈를 줄입니다.
    ///
    /// # 인자
    /// * `img` - 그레이스케일(또는 단일 채널) 영상
    ///
    /// # 반환
    /// * 블러가 적용된 영상
    ///
    /// # Rust 문법 장점
    /// - `?`를 통해 에러가 자동 전파되므로, 각 단계별 에러 처리를 간결히 표현 가능합니다.
    pub(crate) fn noise_removal(&self, img: &UMat) -> Result<UMat> {
        let mut dst = UMat::new_def();
        let kernel = Size::new(self.gaussian_kernel.0, self.gaussian_kernel.1);
        imgproc::gaussian_blur(
            img,
            &mut dst,
            kernel,
            self.gaussian_sigma.0,
            self.gaussian_sigma.1,
            core::BORDER_DEFAULT,
            ALGO_HINT_DEFAULT,
        )?;
        Ok(dst)
    }

    /// 캐니(Canny) 엣지 검출을 수행합니다.
    ///
    /// # 인자
    /// * `img` - 그레이(또는 블러된) 영상
    ///
    /// # 반환
    /// * 엣지를 나타내는 이진(0/255) 영상
    pub(crate) fn edge_detection(&self, img: &UMat) -> Result<UMat> {
        let mut edges = UMat::new_def();
        imgproc::canny(
            img,
            &mut edges,
            self.canny_low_threshold,
            self.canny_high_threshold,
            self.canny_aperture_size,
            false,
        )?;
        Ok(edges)
    }

    /// 모폴로지 닫힘(Closing) 연산을 적용하여 엣지 사이 간격을 메우고 잡음 제거.
    ///
    /// # 인자
    /// * `img` - 이진(엣지) 영상
    ///
    /// # 반환
    /// * 닫힘(Closing) 처리된 영상
    pub(crate) fn morphology_close(&self, img: &UMat) -> Result<UMat> {
        let (rows, cols) = self.morphology_kernel_size;
        let kernel = Mat::ones(rows, cols, CV_8UC1)?.to_mat()?;
        let mut dst = UMat::new_def();
        imgproc::morphology_ex(
            img,
            &mut dst,
            imgproc::MORPH_CLOSE,
            &kernel,
            Point::new(-1, -1),
            self.morphology_iterations,
            core::BORDER_CONSTANT,
            Scalar::default(),
        )?;
        Ok(dst)
    }

    pub(crate) fn mat_to_umat(&self, img: &Mat) -> Result<UMat> {
        let mut dst = UMat::new_def();
        img.copy_to(&mut dst)?;
        Ok(dst)
    }

    pub(crate) fn umat_to_mat(&self, img: &UMat) -> Result<Mat> {
        let mut dst = Mat::default();
        img.copy_to(&mut dst)?;
        Ok(dst)
    }

    /// 관심영역(ROI)만 남기고 외부 영역을 제거합니다.
    ///
    /// # 인자
    /// * `img` - 이진 영상(또는 다채널)에서 ROI를 적용할 대상
    ///
    /// # 반환
    /// * ROI가 적용된 영상
    ///
    /// # Rust 문법 장점
    /// - `Vec<Point>`를 사용해 동적 크기의 꼭짓점 목록을 관리하고,
    ///   OpenCV에 넘길 때도 `.as_slice()` 등으로 안전하게 참조가 가능합니다.
    pub(crate) fn roi(&self, img: &Mat) -> Result<Mat> {
        // 영상과 동일한 크기의 검정색 마스크 생성
        let mut mask = Mat::zeros(img.rows(), img.cols(), img.typ())?.to_mat()?;
        {
            // self.vertices(4개 점)을 Mat 형태로 변환
            let mat_boxed = Mat::from_slice(&self.vertices)?;
            let sub_mat_box = mat_boxed.reshape(2, self.vertices.len() as i32)?;
            let sub_mat = sub_mat_box.try_clone()?;

            let mut contour_vec: Vector<Mat> = Vector::new();
            contour_vec.push(sub_mat);

            // 마스크에 흰색으로 폴리곤(ROI) 영역을 채움
            imgproc::fill_poly(
                &mut mask,
                &contour_vec,
                Scalar::new(255.0, 255.0, 255.0, 255.0),
                imgproc::LINE_8,
                0,
                Point::new(0, 0),
            )?;
        }

        // mask와 bitwise_and를 하여 ROI 내부만 살림
        let mut masked_img = Mat::default();
        core::bitwise_and(img, &mask, &mut masked_img, &Mat::default())?;
        Ok(masked_img)
    }

    /// 입력 이미지를 Bird's-eye(탑뷰)로 투시 변환합니다.
    ///
    /// # 인자
    /// * `img` - ROI를 이미 적용한 이진(or 그레이 등) 영상
    ///
    /// # 반환
    /// * 탑뷰로 변환된 결과 영상
    pub(crate) fn perspective_transform(&self, img: &Mat) -> Result<Mat> {
        let mut result = Mat::default();
        imgproc::warp_perspective(
            img,
            &mut result,
            &self.transform_matrix,
            Size::new(self.width, self.height),
            imgproc::INTER_LINEAR,
            core::BORDER_CONSTANT,
            Scalar::default(),
        )?;
        Ok(result)
    }

    /// Bird's-eye로 변환된 영상을 다시 원근 시야로 역투시 변환합니다.
    ///
    /// # 인자
    /// * `img` - 탑뷰 상태의 영상
    ///
    /// # 반환
    /// * 원근 시야(원본 좌표)에 대응하는 영상
    fn inv_perspective_transform(&self, img: &Mat) -> Result<Mat> {
        let mut result = Mat::default();
        imgproc::warp_perspective(
            img,
            &mut result,
            &self.inv_transform_matrix,
            Size::new(self.width, self.height),
            imgproc::INTER_LINEAR,
            core::BORDER_CONSTANT,
            Scalar::default(),
        )?;
        Ok(result)
    }

    /// **슬라이딩 윈도우** 방식을 이용하여 차선을 탐지합니다.
    ///
    /// 주요 단계:
    /// 1. 이진 영상에서 0이 아닌 픽셀의 (x,y) 좌표를 모두 찾음.
    /// 2. 화면을 수직 방향으로 `nwindows` 개의 윈도우로 나누어, 각 윈도우에서
    ///    왼쪽/오른쪽 차선 픽셀을 선택.
    /// 3. 기준 픽셀 수가 충분하면 평균 x좌표로 윈도우 중심을 이동.
    /// 4. 수집된 픽셀들로 2차 곡선(Polyfit) 추정.
    ///
    /// # 반환
    /// 1. 슬라이딩 윈도우 과정을 시각화한 3채널 Mat (`out_img`)
    /// 2. 왼쪽 차선 x좌표 곡선(`left_fitx`)
    /// 3. 오른쪽 차선 x좌표 곡선(`right_fitx`)
    /// 4. 왼쪽 차선 인식 여부(`left_lane_detected`)
    /// 5. 오른쪽 차선 인식 여부(`right_lane_detected`)
    ///
    /// # 매개변수
    /// * `binary_img` : 차선 탐색을 수행할 이진 영상
    /// * `nwindows` : 세로 방향 윈도우 개수
    /// * `margin` : 윈도우 좌우 폭
    /// * `minpix` : 윈도우 내 픽셀 개수가 이 값 이상이면 중앙좌표 갱신
    /// * `draw_windows` : 탐색 윈도우 시각화 여부
    ///
    /// # Rust-C++ 비교
    /// - `Vec<(i32, i32)>`로 픽셀 좌표를 저장하며, 이 역시 안전하게
    ///   관리가 가능합니다. C/C++에서는 할당/해제 시점 등을 직접 처리해야 하지만,
    ///   Rust에선 자동으로 수행됩니다(RAII).
    pub fn sliding_window(
        &mut self,
        binary_img: &Mat,
        nwindows: i32,
        margin: i32,
        minpix: i32,
        draw_windows: bool,
    ) -> Result<(Mat, Vec<f64>, Vec<f64>, bool, bool)> {
        // -------------------------------------------
        // 1) row별로 nonzero x좌표를 미리 분류
        // -------------------------------------------
        let nonzero_points_by_row = get_nonzero_points_by_row(binary_img)?;

        // -------------------------------------------
        // 2) 시각화용 out_img, window_img 준비
        // -------------------------------------------
        let mut out_img =
            Mat::new_size_with_default(binary_img.size()?, CV_8UC3, Scalar::all(0.0))?;

        // 이진 영상을 3채널로 변환해 out_img에 합치기
        {
            use opencv::core::Vector;
            let mut channels: Vector<Mat> = Vector::new();
            channels.push(binary_img.clone());
            channels.push(binary_img.clone());
            channels.push(binary_img.clone());
            core::merge(&channels, &mut out_img)?;
        }

        // 시각화용 window_img: (height x (width + 2*margin))
        let mut window_img =
            Mat::zeros(self.height, self.width + 2 * self.window_margin, CV_8UC3)?.to_mat()?;

        // out_img를 window_img 중앙 영역에 복사
        {
            let roi = core::Rect::new(self.window_margin, 0, self.width, self.height);
            let mut window_roi = window_img.roi(roi)?.try_clone()?;
            out_img.copy_to(&mut window_roi)?;
        }

        // -------------------------------------------
        // 3) 슬라이딩 윈도우 매개변수
        // -------------------------------------------
        let midpoint = self.width / 2;
        let window_height = self.height / nwindows;

        // 현재 (왼/오) x좌표
        let mut leftx_current = self.leftx_base;
        let mut rightx_current = self.rightx_base;

        // 한쪽이 픽셀 없을 때 다른 쪽 따라가기 위해 이전 좌표 보관
        let mut leftx_past = leftx_current;
        let mut rightx_past = rightx_current;

        // 윈도우 탐색 후, 왼/오 차선 픽셀 (y, x) 목록
        let mut left_lane_points: Vec<(i32, i32)> = Vec::new();
        let mut right_lane_points: Vec<(i32, i32)> = Vec::new();

        // -------------------------------------------
        // 4) 윈도우 탐색: 아래에서 위로(nwindows 번)
        // -------------------------------------------
        for window_i in 0..nwindows {
            let win_y_low = self.height - (window_i + 1) * window_height;
            let win_y_high = self.height - window_i * window_height;

            let win_xleft_low = leftx_current - margin;
            let win_xleft_high = leftx_current + margin;
            let win_xright_low = rightx_current - margin;
            let win_xright_high = rightx_current + margin;

            // (4-1) 시각화용 사각형 그리기
            if draw_windows {
                // 왼쪽 윈도우 사각형
                imgproc::rectangle(
                    &mut out_img,
                    core::Rect::new(
                        win_xleft_low,
                        win_y_low,
                        (win_xleft_high - win_xleft_low).max(0),
                        (win_y_high - win_y_low).max(0),
                    ),
                    Scalar::new(0.0, 255.0, 0.0, 255.0),
                    2,
                    imgproc::LINE_8,
                    0,
                )?;
                // 오른쪽 윈도우 사각형
                imgproc::rectangle(
                    &mut out_img,
                    core::Rect::new(
                        win_xright_low,
                        win_y_low,
                        (win_xright_high - win_xright_low).max(0),
                        (win_y_high - win_y_low).max(0),
                    ),
                    Scalar::new(0.0, 255.0, 0.0, 255.0),
                    2,
                    imgproc::LINE_8,
                    0,
                )?;

                // window_img에도 연분홍색으로 사각형
                imgproc::rectangle(
                    &mut window_img,
                    core::Rect::new(
                        win_xleft_low + self.window_margin,
                        win_y_low,
                        (win_xleft_high - win_xleft_low).max(0),
                        (win_y_high - win_y_low).max(0),
                    ),
                    Scalar::new(255.0, 100.0, 100.0, 255.0),
                    1,
                    imgproc::LINE_8,
                    0,
                )?;
                imgproc::rectangle(
                    &mut window_img,
                    core::Rect::new(
                        win_xright_low + self.window_margin,
                        win_y_low,
                        (win_xright_high - win_xright_low).max(0),
                        (win_y_high - win_y_low).max(0),
                    ),
                    Scalar::new(255.0, 100.0, 100.0, 255.0),
                    1,
                    imgproc::LINE_8,
                    0,
                )?;
            }

            // (4-2) 이 윈도우 범위 안의 픽셀 찾기 (row 단위)
            let mut good_left_count = 0;
            let mut good_right_count = 0;

            for y in win_y_low..win_y_high {
                if y < 0 || y >= self.height {
                    continue;
                }
                // 해당 row에 있는 nonzero x좌표들
                let row_nonzeros = &nonzero_points_by_row[y as usize];

                // 왼쪽 윈도우 범위
                let left_in_range = row_nonzeros
                    .iter()
                    .filter(|&&x| x >= win_xleft_low && x < win_xleft_high);
                for &x in left_in_range {
                    left_lane_points.push((y, x));
                    good_left_count += 1;
                }

                // 오른쪽 윈도우 범위
                let right_in_range = row_nonzeros
                    .iter()
                    .filter(|&&x| x >= win_xright_low && x < win_xright_high);
                for &x in right_in_range {
                    right_lane_points.push((y, x));
                    good_right_count += 1;
                }
            }

            // (4-3) 픽셀이 일정 이상이면, 그 픽셀들의 평균 x좌표를 새 중심으로
            if good_left_count as i32 > minpix {
                let sum_x: i32 = left_lane_points
                    .iter()
                    .rev()
                    .take(good_left_count as usize)
                    .map(|&(_y, x)| x)
                    .sum();
                let mean_x = sum_x as f64 / good_left_count as f64;
                leftx_current = mean_x as i32;
            }
            if good_right_count as i32 > minpix {
                let sum_x: i32 = right_lane_points
                    .iter()
                    .rev()
                    .take(good_right_count as usize)
                    .map(|&(_y, x)| x)
                    .sum();
                let mean_x = sum_x as f64 / good_right_count as f64;
                rightx_current = mean_x as i32;
            }

            // (4-4) 한쪽만 픽셀이 부족하면, 다른 쪽 이동량 따라감
            if (good_left_count as i32) < minpix {
                leftx_current = leftx_current + (rightx_current - rightx_past);
            }
            if (good_right_count as i32) < minpix {
                rightx_current = rightx_current + (leftx_current - leftx_past);
            }

            // (4-5) 첫 윈도우에서 기준점 갱신
            if window_i == 0 {
                if leftx_current > midpoint + 40 {
                    leftx_current = midpoint + 40;
                }
                if leftx_current < 0 {
                    leftx_current = 0;
                }
                if rightx_current < midpoint - 40 {
                    rightx_current = midpoint - 40;
                }
                if rightx_current > self.width {
                    rightx_current = self.width;
                }
                self.leftx_base = leftx_current;
                self.rightx_base = rightx_current;
            }

            leftx_past = leftx_current;
            rightx_past = rightx_current;
        }

        // -------------------------------------------
        // 5) 최종 (x,y) 좌표 분리 & 차선 픽셀 개수로 인식 여부 판정
        // -------------------------------------------
        let (mut leftx_vals, mut lefty_vals) = (Vec::new(), Vec::new());
        for &(y, x) in &left_lane_points {
            leftx_vals.push(x as f64);
            lefty_vals.push(y as f64);
        }
        let (mut rightx_vals, mut righty_vals) = (Vec::new(), Vec::new());
        for &(y, x) in &right_lane_points {
            rightx_vals.push(x as f64);
            righty_vals.push(y as f64);
        }

        let left_lane_detected = leftx_vals.len() >= self.lane_points_threshold;
        let right_lane_detected = rightx_vals.len() >= self.lane_points_threshold;

        // -------------------------------------------
        // 6) 2차 곡선 피팅
        // -------------------------------------------
        if left_lane_detected {
            if let Some(fit) = polyfit_2d(&lefty_vals, &leftx_vals) {
                self.left_a.push(fit[0]);
                self.left_b.push(fit[1]);
                self.left_c.push(fit[2]);
            }
        }
        if right_lane_detected {
            if let Some(fit) = polyfit_2d(&righty_vals, &rightx_vals) {
                self.right_a.push(fit[0]);
                self.right_b.push(fit[1]);
                self.right_c.push(fit[2]);
            }
        }

        // -------------------------------------------
        // 7) 마스크를 이용해 픽셀 색칠 (out_img, window_img)
        // -------------------------------------------
        let binary_img_rows = binary_img.rows();
        let binary_img_cols = binary_img.cols();

        let mut left_mask = Mat::zeros(binary_img_rows, binary_img_cols, CV_8UC1)?.to_mat()?;
        let mut right_mask = Mat::zeros(binary_img_rows, binary_img_cols, CV_8UC1)?.to_mat()?;

        // 왼쪽/오른쪽 차선 픽셀 좌표에 255 할당
        for &(y, x) in &left_lane_points {
            if y >= 0 && y < self.height && x >= 0 && x < self.width {
                *left_mask.at_2d_mut::<u8>(y, x)? = 255;
            }
        }
        for &(y, x) in &right_lane_points {
            if y >= 0 && y < self.height && x >= 0 && x < self.width {
                *right_mask.at_2d_mut::<u8>(y, x)? = 255;
            }
        }

        // out_img에 왼쪽=파랑, 오른쪽=빨강 칠하기
        out_img.set_to(&Scalar::new(255.0, 0.0, 0.0, 255.0), &left_mask)?;
        out_img.set_to(&Scalar::new(0.0, 0.0, 255.0, 255.0), &right_mask)?;

        // window_img에 마스크를 margin만큼 x좌표 shift해서 칠하기
        let window_img_rows = window_img.rows();
        let window_img_cols = window_img.cols();

        let mut left_mask_w = Mat::zeros(window_img_rows, window_img_cols, CV_8UC1)?.to_mat()?;
        let mut right_mask_w = Mat::zeros(window_img_rows, window_img_cols, CV_8UC1)?.to_mat()?;

        for &(y, x) in &left_lane_points {
            let x_shifted = x + self.window_margin;
            if y >= 0
                && y < self.height
                && x_shifted >= 0
                && x_shifted < (self.width + 2 * self.window_margin)
            {
                *left_mask_w.at_2d_mut::<u8>(y, x_shifted)? = 255;
            }
        }
        for &(y, x) in &right_lane_points {
            let x_shifted = x + self.window_margin;
            if y >= 0
                && y < self.height
                && x_shifted >= 0
                && x_shifted < (self.width + 2 * self.window_margin)
            {
                *right_mask_w.at_2d_mut::<u8>(y, x_shifted)? = 255;
            }
        }

        window_img.set_to(&Scalar::new(255.0, 0.0, 0.0, 255.0), &left_mask_w)?;
        window_img.set_to(&Scalar::new(0.0, 0.0, 255.0, 255.0), &right_mask_w)?;

        // -------------------------------------------
        // 8) 최근 10개 계수 평균 내서 안정화
        // -------------------------------------------
        let left_fit_avg = [
            mean_of_last_10(&self.left_a),
            mean_of_last_10(&self.left_b),
            mean_of_last_10(&self.left_c),
        ];
        let right_fit_avg = [
            mean_of_last_10(&self.right_a),
            mean_of_last_10(&self.right_b),
            mean_of_last_10(&self.right_c),
        ];

        // -------------------------------------------
        // 9) ploty에 따라 최종 x좌표 (ax^2 + bx + c)
        // -------------------------------------------
        let left_fitx: Vec<f64> = self
            .ploty
            .par_iter()
            .map(|&yv| left_fit_avg[0] * yv * yv + left_fit_avg[1] * yv + left_fit_avg[2])
            .collect();
        let right_fitx: Vec<f64> = self
            .ploty
            .par_iter()
            .map(|&yv| right_fit_avg[0] * yv * yv + right_fit_avg[1] * yv + right_fit_avg[2])
            .collect();

        // -------------------------------------------
        // 10) 양쪽 다 인식 안 되면 기준점 리셋
        // -------------------------------------------
        if !left_lane_detected && !right_lane_detected {
            self.leftx_base = self.leftx_mid - 30;
            self.rightx_base = self.rightx_mid + 30;
        }

        // 반환
        Ok((
            out_img,
            left_fitx,
            right_fitx,
            left_lane_detected,
            right_lane_detected,
        ))
    }
    /// **이미 검출된 차선 주변에서만 픽셀을 탐색하여 연산 속도를 높입니다.**
    ///
    /// 이 함수는 `sliding_window`와 달리, 이전 프레임에서 성공적으로
    /// 찾은 차선 다항식(polynomial) 주변의 'margin' 영역만 탐색합니다.
    ///
    /// # 주요 단계
    /// 1. 이전 프레임들로부터 계산된 평균 차선 곡선(fit)을 가져옴.
    /// 2. 이미지의 모든 행(y)에 대해, 이 곡선 주변 `margin` 내에 있는
    ///    0이 아닌 픽셀만 수집.
    /// 3. 수집된 픽셀들로 새로운 2차 곡선(Polyfit)을 추정.
    ///
    /// # 반환
    /// 1. 탐색 영역과 검출된 차선을 시각화한 3채널 Mat (`out_img`)
    /// 2. 왼쪽 차선 x좌표 곡선(`left_fitx`)
    /// 3. 오른쪽 차선 x좌표 곡선(`right_fitx`)
    /// 4. 왼쪽 차선 인식 여부(`left_lane_detected`)
    /// 5. 오른쪽 차선 인식 여부(`right_lane_detected`)
    ///
    /// # 매개변수
    /// * `binary_img` : 차선 탐색을 수행할 이진 영상
    /// * `margin` : 기존 차선 곡선 주변으로 탐색할 좌우 폭
    pub fn search_around_poly(
        &mut self,
        binary_img: &Mat,
        margin: i32,
    ) -> Result<(Mat, Vec<f64>, Vec<f64>, bool, bool)> {
        // -------------------------------------------
        // 1) 이전 프레임들의 차선 계수 평균값 가져오기
        // -------------------------------------------
        let left_fit_avg = [
            mean_of_last_10(&self.left_a),
            mean_of_last_10(&self.left_b),
            mean_of_last_10(&self.left_c),
        ];
        let right_fit_avg = [
            mean_of_last_10(&self.right_a),
            mean_of_last_10(&self.right_b),
            mean_of_last_10(&self.right_c),
        ];

        let (height, width) = (binary_img.rows(), binary_img.cols());
        let width_i32 = width;
        let step = binary_img.step1(0)? as usize;
        let data = binary_img.data_bytes()?;

        // -------------------------------------------
        // 3) 이전 차선 주변의 픽셀들만 선택
        // -------------------------------------------
        let (left_lane_points, right_lane_points) = (0..height)
            .into_par_iter()
            .map(|y| {
                let y_f64 = y as f64;
                let leftx_center =
                    left_fit_avg[0] * y_f64 * y_f64 + left_fit_avg[1] * y_f64 + left_fit_avg[2];
                let rightx_center =
                    right_fit_avg[0] * y_f64 * y_f64 + right_fit_avg[1] * y_f64 + right_fit_avg[2];

                let left_search_low = (leftx_center - margin as f64).floor() as i32;
                let left_search_high = (leftx_center + margin as f64).ceil() as i32;
                let right_search_low = (rightx_center - margin as f64).floor() as i32;
                let right_search_high = (rightx_center + margin as f64).ceil() as i32;

                let row_start = y as usize * step;
                let row_end = (row_start + step).min(data.len());
                let row_slice = &data[row_start..row_end];
                let usable_width = width_i32.min((row_end - row_start) as i32);

                let mut left_lane_local: Vec<(i32, i32)> = Vec::new();
                let mut right_lane_local: Vec<(i32, i32)> = Vec::new();

                let left_low = left_search_low.clamp(0, usable_width);
                let left_high = left_search_high.clamp(0, usable_width);
                if left_low < left_high {
                    let slice = &row_slice[left_low as usize..left_high as usize];
                    for (offset, &pixel) in slice.iter().enumerate() {
                        if pixel != 0 {
                            left_lane_local.push((y, left_low + offset as i32));
                        }
                    }
                }

                let right_low = right_search_low.clamp(0, usable_width);
                let right_high = right_search_high.clamp(0, usable_width);
                if right_low < right_high {
                    let slice = &row_slice[right_low as usize..right_high as usize];
                    for (offset, &pixel) in slice.iter().enumerate() {
                        if pixel != 0 {
                            right_lane_local.push((y, right_low + offset as i32));
                        }
                    }
                }

                (left_lane_local, right_lane_local)
            })
            .reduce(
                || (Vec::new(), Vec::new()),
                |mut acc, (mut left_local, mut right_local)| {
                    acc.0.append(&mut left_local);
                    acc.1.append(&mut right_local);
                    acc
                },
            );

        // -------------------------------------------
        // 4) 최종 (x,y) 좌표 분리 & 차선 픽셀 개수로 인식 여부 판정
        // -------------------------------------------
        let leftx_vals: Vec<f64> = left_lane_points
            .par_iter()
            .map(|&(_, x)| x as f64)
            .collect();
        let lefty_vals: Vec<f64> = left_lane_points
            .par_iter()
            .map(|&(y, _)| y as f64)
            .collect();
        let rightx_vals: Vec<f64> = right_lane_points
            .par_iter()
            .map(|&(_, x)| x as f64)
            .collect();
        let righty_vals: Vec<f64> = right_lane_points
            .par_iter()
            .map(|&(y, _)| y as f64)
            .collect();

        let left_lane_detected = leftx_vals.len() >= self.lane_points_threshold;
        let right_lane_detected = rightx_vals.len() >= self.lane_points_threshold;

        // -------------------------------------------
        // 5) 새로운 2차 곡선 피팅 (현재 프레임 기준)
        // -------------------------------------------
        if left_lane_detected {
            if let Some(fit) = polyfit_2d(&lefty_vals, &leftx_vals) {
                self.left_a.push(fit[0]);
                self.left_b.push(fit[1]);
                self.left_c.push(fit[2]);
            }
        }
        if right_lane_detected {
            if let Some(fit) = polyfit_2d(&righty_vals, &rightx_vals) {
                self.right_a.push(fit[0]);
                self.right_b.push(fit[1]);
                self.right_c.push(fit[2]);
            }
        }

        // -------------------------------------------
        // 6) 최근 계수 평균으로 안정화 및 최종 x좌표 계산
        // -------------------------------------------
        let new_left_fit_avg = [
            mean_of_last_10(&self.left_a),
            mean_of_last_10(&self.left_b),
            mean_of_last_10(&self.left_c),
        ];
        let new_right_fit_avg = [
            mean_of_last_10(&self.right_a),
            mean_of_last_10(&self.right_b),
            mean_of_last_10(&self.right_c),
        ];

        let left_fitx: Vec<f64> = self
            .ploty
            .par_iter()
            .map(|&yv| {
                new_left_fit_avg[0] * yv * yv + new_left_fit_avg[1] * yv + new_left_fit_avg[2]
            })
            .collect();
        let right_fitx: Vec<f64> = self
            .ploty
            .par_iter()
            .map(|&yv| {
                new_right_fit_avg[0] * yv * yv + new_right_fit_avg[1] * yv + new_right_fit_avg[2]
            })
            .collect();

        // -------------------------------------------
        // 7) 결과 시각화
        // -------------------------------------------
        let mut out_img = Mat::default();
        imgproc::cvt_color(
            &binary_img,
            &mut out_img,
            imgproc::COLOR_GRAY2BGR,
            0,
            ALGO_HINT_DEFAULT,
        )?;

        let mut window_img = Mat::zeros(height, width, CV_8UC3)?.to_mat()?;

        let mut left_line_pts_inner: Vector<core::Point> = Vector::new();
        let mut left_line_pts_outer: Vector<core::Point> = Vector::new();
        let mut right_line_pts_inner: Vector<core::Point> = Vector::new();
        let mut right_line_pts_outer: Vector<core::Point> = Vector::new();

        for i in 0..self.ploty.len() {
            let y = self.ploty[i] as i32;
            left_line_pts_inner.push(core::Point::new(left_fitx[i] as i32 - margin, y));
            left_line_pts_outer.push(core::Point::new(left_fitx[i] as i32 + margin, y));
            right_line_pts_inner.push(core::Point::new(right_fitx[i] as i32 - margin, y));
            right_line_pts_outer.push(core::Point::new(right_fitx[i] as i32 + margin, y));
        }

        let mut left_pts: Vector<core::Point> = Vector::new();
        left_pts.extend(left_line_pts_inner.iter());
        left_pts.extend(left_line_pts_outer.iter().rev());

        let mut right_pts: Vector<core::Point> = Vector::new();
        right_pts.extend(right_line_pts_inner.iter());
        right_pts.extend(right_line_pts_outer.iter().rev());

        // [수정된 부분] `Vec<Vector<...>>` 대신 `Vector<Vector<...>>` 사용
        let mut left_polygons: Vector<Vector<core::Point>> = Vector::new();
        left_polygons.push(left_pts);
        let mut right_polygons: Vector<Vector<core::Point>> = Vector::new();
        right_polygons.push(right_pts);

        imgproc::fill_poly(
            &mut window_img,
            &left_polygons, // 올바른 타입 전달
            Scalar::new(0.0, 255.0, 0.0, 255.0),
            imgproc::LINE_8,
            0,
            core::Point::new(0, 0),
        )?;
        imgproc::fill_poly(
            &mut window_img,
            &right_polygons, // 올바른 타입 전달
            Scalar::new(0.0, 255.0, 0.0, 255.0),
            imgproc::LINE_8,
            0,
            core::Point::new(0, 0),
        )?;

        let out_img_src = out_img.clone();
        core::add_weighted(&out_img_src, 1.0, &window_img, 0.3, 0.0, &mut out_img, -1)?;

        // 검출된 차선 픽셀 색칠
        for &(y, x) in &left_lane_points {
            if y >= 0 && y < height && x >= 0 && x < width {
                *out_img.at_2d_mut::<core::Vec3b>(y, x)? = core::Vec3b::from([255, 0, 0]);
            }
        }
        for &(y, x) in &right_lane_points {
            if y >= 0 && y < height && x >= 0 && x < width {
                *out_img.at_2d_mut::<core::Vec3b>(y, x)? = core::Vec3b::from([0, 0, 255]);
            }
        }

        // -------------------------------------------
        // 8) 양쪽 다 인식 안 되면 기준점 리셋 (Sliding Window 재시도 신호)
        // -------------------------------------------
        if !left_lane_detected && !right_lane_detected {
            self.leftx_base = self.leftx_mid - 30;
            self.rightx_base = self.rightx_mid + 30;
        }

        Ok((
            out_img,
            left_fitx,
            right_fitx,
            left_lane_detected,
            right_lane_detected,
        ))
    }

    /// 검출된 차선(왼/오)을 토대로 조향각을 계산합니다.
    ///
    /// **기본 로직**:
    /// 1) 양쪽 다 인식이 안 되면 이전 각도 유지
    /// 2) 한쪽만 인식되면 해당 차선을 1차 직선으로 피팅하여 기울기 계산
    /// 3) 양쪽 모두 인식되면 각각 1차 피팅 → 교점(소실점) 찾은 뒤 기울기
    /// 4) 최종 각도 = `atan(기울기)*180/π`. 여기서 0도를 "정면" 기준으로 맞추기 위해
    ///    90도에서 오프셋 조정.
    ///
    /// # 반환
    /// - `steering_angle`: 새로 계산된 조향각(도 단위)
    pub(crate) fn get_angle_on_lane(
        &mut self,
        left_fitx: &[f64],
        right_fitx: &[f64],
        left_lane_detected: bool,
        right_lane_detected: bool,
    ) -> f64 {
        // 양쪽 다 인식 실패 시 이전 각도 그대로
        if !left_lane_detected && !right_lane_detected {
            return self.prev_angle;
        }

        let slope: f64; // 최종 기울기

        // 한쪽만 인식된 경우
        if (left_lane_detected && !right_lane_detected)
            || (!left_lane_detected && right_lane_detected)
        {
            if left_lane_detected {
                // 왼쪽만 인식 => 1차 피팅
                if let Some([a, _b]) = polyfit_1d(left_fitx, &self.ploty) {
                    slope = a;
                } else {
                    slope = 0.0;
                }
            } else {
                // 오른쪽만 인식 => 1차 피팅
                if let Some([a, _b]) = polyfit_1d(right_fitx, &self.ploty) {
                    slope = a;
                } else {
                    slope = 0.0;
                }
            }
        } else {
            // 양쪽 다 인식된 경우
            let [left_a, left_b] = if let Some([a, b]) = polyfit_1d(left_fitx, &self.ploty) {
                [a, b]
            } else {
                [0.0, 0.0]
            };

            let [right_a, right_b] = if let Some([a, b]) = polyfit_1d(right_fitx, &self.ploty) {
                [a, b]
            } else {
                [0.0, 0.0]
            };

            // 두 직선의 기울기가 거의 같으면 (평행 판단)
            if (left_a - right_a).abs() < 1e-12 {
                let inter_x = -(left_b + right_b) / (2.0 * left_a);
                let inter_y = 0.0;
                slope = (self.height as f64 - inter_y) / ((self.width as f64 / 2.0) - inter_x);
            } else {
                // 교점 = (b2 - b1) / (a1 - a2)
                let inter_x = (right_b - left_b) / (left_a - right_a);
                let inter_y = left_a * inter_x + left_b;
                slope = (self.height as f64 - inter_y) / ((self.width as f64 / 2.0) - inter_x);
            }
        }

        // 기울기를 각도로 변환 (atan -> deg)
        let mut steering_angle = slope.atan() * 180.0 / PI;

        // 0도 기준을 위해 90도에서 오프셋 조정
        if steering_angle > 0.0 {
            steering_angle -= 90.0;
            if steering_angle <= -20.0 {
                steering_angle = -20.0;
            }
        } else if steering_angle < 0.0 {
            steering_angle += 90.0;
            if steering_angle >= 20.0 {
                steering_angle = 20.0;
            }
        }

        self.prev_angle = steering_angle;
        self.steering_angle = steering_angle;
        steering_angle
    }

    /// 1차원 칼만 필터로 조향각 측정값을 평활화합니다.
    ///
    /// # Arguments
    /// * `measurement` - 새로 측정된 조향각(도 단위)
    ///
    /// # Returns
    /// 업데이트된 추정 조향각.
    pub fn update_angle_kalman(&mut self, measurement: f64) -> f64 {
        let predicted_cov = self.kalman_covariance + self.kalman_process_noise;
        let kalman_gain = predicted_cov / (predicted_cov + self.kalman_measurement_noise);
        self.kalman_estimate += kalman_gain * (measurement - self.kalman_estimate);
        self.kalman_covariance = (1.0 - kalman_gain) * predicted_cov;
        self.kalman_estimate
    }

    /// 칼만 필터 상태를 재설정합니다.
    pub fn reset_kalman(&mut self, initial_estimate: f64, initial_uncertainty: f64) {
        self.kalman_estimate = initial_estimate;
        self.kalman_covariance = initial_uncertainty.max(1e-6);
    }

    /// 계산된 조향각을 이용해 영상 위에 하나의 직선을 그려줍니다.
    ///
    /// # 인자
    /// * `base_img` - 배경이 되는 영상
    /// * `overlay_img` - 합성할 (또는 가중 합성할) 추가 영상
    /// * `steering_angle` - 조향각 (도 단위)
    ///
    /// # 반환
    /// * 조향각 직선이 그려진 최종 영상
    fn display_heading_line(
        &self,
        base_img: &Mat,
        overlay_img: &Mat,
        steering_angle: f64,
    ) -> Result<Mat> {
        let mut img_clone = base_img.clone();
        let (height, width) = (img_clone.rows(), img_clone.cols());
        if height <= 0 || width <= 0 {
            return Ok(img_clone);
        }

        // Rust의 match를 사용하지 않았지만, angle 조정에 if를 활용
        // (C++에서 if-else와 다르지 않아 보이지만, Rust에선 match 구문으로도 간결하게 표현 가능)
        let mut angle = steering_angle;
        if angle > 0.0 {
            angle += 90.0;
        } else if angle < 0.0 {
            angle -= 90.0;
        } else {
            angle = 90.0;
        }

        // 각도를 라디안으로 변환
        let rad = angle * PI / 180.0;

        // 시작점: 화면 하단 중앙
        let x1 = (width / 2) as i32;
        let y1 = height as i32;

        // 끝점: 일정 길이 위로 올라간 지점
        let x2 = (x1 as f64 - (self.height as f64 / 2.0) / rad.tan()) as i32;
        let y2 = (self.height as f64 * 3.5 / 5.0) as i32;

        // 굵은 초록색 선
        imgproc::line(
            &mut img_clone,
            Point::new(x1, y1),
            Point::new(x2, y2),
            Scalar::new(0.0, 255.0, 0.0, 255.0),
            5,
            imgproc::LINE_8,
            0,
        )?;

        // 두 이미지를 가중 합성
        let mut heading_image = Mat::default();
        core::add_weighted(
            &img_clone,
            1.0,
            overlay_img,
            1.0,
            0.0,
            &mut heading_image,
            -1,
        )?;
        Ok(heading_image)
    }

    /// 단일 프레임에 대한 전체 차선 인식 처리 과정입니다.
    ///
    /// 1) 그레이 변환
    /// 2) 블러(노이즈 제거)
    /// 3) 캐니 엣지
    /// 4) 모폴로지 닫힘
    /// 5) ROI
    /// 6) Bird's-eye 투시 변환
    /// 7) 슬라이딩 윈도우 → 차선 검출
    /// 8) 조향각 계산
    /// 9) 조향각 시각화
    /// 10) 역투시 변환
    /// 11) 원본과 합성
    ///
    /// 마지막으로 좌우 영상을 합쳐 하나의 결과 영상을 반환합니다.
    ///
    /// # Rust 문법 장점
    /// - 함수가 길더라도 `?` 연산자를 통해 에러 처리를 단순화할 수 있어
    ///   가독성을 유지할 수 있습니다.
    pub fn processing(&mut self, frame: &Mat) -> Result<Mat> {
        let start_time = Instant::now();

        let img = frame.clone();

        // 1) 그레이 변환
        let gray = self.gray_scale(&img)?;

        // 2) 가우시안 블러
        let blur = self.noise_removal(&gray)?;

        // 3) 캐니 엣지
        let edges = self.edge_detection(&blur)?;

        // 4) 모폴로지 닫힘
        let closed = self.morphology_close(&edges)?;
        let closed_mat = self.umat_to_mat(&closed)?;

        // 5) ROI
        let roi_img = self.roi(&closed_mat)?;

        // 6) Bird's-eye 투시 변환
        let birds_eye = self.perspective_transform(&roi_img)?;

        // 7) 슬라이딩 윈도우
        let sliding_window_img: Mat;
        let left_fitx: Vec<f64>;
        let right_fitx: Vec<f64>;
        let left_lane_detected: bool;
        let right_lane_detected: bool;

        // 조건: 이전에 차선을 성공적으로 찾은 기록이 있는가?
        let has_previous_detection = !self.left_a.is_empty() && !self.right_a.is_empty();

        if has_previous_detection {
            // [빠른 추적] 이전 기록이 있으면, 그 주변만 빠르게 탐색합니다.
            // println!("✅ 빠른 추적 모드: search_around_poly 실행");
            let (img, lfx, rfx, l_det, r_det) =
                self.search_around_poly(&birds_eye, self.search_poly_margin)?;
            println!(
                "✅ 빠른 추적 실행: Left Pixels = {}, Right Pixels = {}, Left Detected = {}, Right Detected = {}",
                lfx.len(),
                rfx.len(),
                l_det,
                r_det
            ); // <--- 로그 추가

            // 결과 할당
            sliding_window_img = img;
            left_fitx = lfx;
            right_fitx = rfx;
            left_lane_detected = l_det;
            right_lane_detected = r_det;

            // [실패 처리] 만약 빠른 추적에 실패했다면, 다음 프레임에서 전체 탐색을 하도록 상태를 리셋합니다.
            if !(left_lane_detected || right_lane_detected) {
                println!("⚠️ 추적 실패! 다음 프레임에서 전체 탐색을 실시합니다.");
                self.left_a.clear();
                self.left_b.clear();
                self.left_c.clear();
                self.right_a.clear();
                self.right_b.clear();
                self.right_c.clear();
            }
        } else {
            // [전체 탐색] 이전 기록이 없으면, sliding_window로 전체 영역을 탐색합니다.
            // println!("🔍 전체 탐색 모드: sliding_window 실행");
            let (img, lfx, rfx, l_det, r_det) = self.sliding_window(
                &birds_eye,
                self.sliding_window_count,
                self.sliding_window_margin,
                self.sliding_window_minpix,
                self.sliding_window_draw_debug,
            )?;
            println!(
                "🔍 전체 탐색 실행: Left Pixels = {}, Right Pixels = {}, Left Detected = {}, Right Detected = {}",
                lfx.len(),
                rfx.len(),
                l_det,
                r_det
            ); // <--- 로그 추가

            // 결과 할당
            sliding_window_img = img;
            left_fitx = lfx;
            right_fitx = rfx;
            left_lane_detected = l_det;
            right_lane_detected = r_det;
        }

        // 8) 조향각 계산
        self.steering_angle = self.get_angle_on_lane(
            &left_fitx,
            &right_fitx,
            left_lane_detected,
            right_lane_detected,
        );

        if self.visible == true {
            // 9) 조향각 시각화 (슬라이딩 윈도우 영상에 선 그리기)
            let sliding_with_line = self.display_heading_line(
                &sliding_window_img,
                &sliding_window_img,
                self.steering_angle,
            )?;

            // 10) 원근 시야로 역투시 변환
            let inv_trans = self.inv_perspective_transform(&sliding_with_line)?;

            // 11) 원본 영상과 합성
            let mut total_processed = Mat::default();
            core::add_weighted(&img, 1.0, &inv_trans, 1.0, 0.0, &mut total_processed, -1)?;

            // 텍스트(조향각) 표시
            let angle_text = format!("Angle: {}", self.steering_angle as i32);
            imgproc::put_text(
                &mut total_processed,
                &angle_text,
                Point::new(20, 100),
                imgproc::FONT_HERSHEY_SIMPLEX,
                2.0,
                Scalar::new(255.0, 255.0, 255.0, 255.0),
                2,
                imgproc::LINE_8,
                false,
            )?;

            // FPS 측정
            let elapsed = start_time.elapsed().as_secs_f32();
            let fps = 1.0 / elapsed;
            let fps_text = format!("FPS: {:.2}", fps);
            let cols = total_processed.cols();
            let text_x = cols - 400;
            let text_y = 50;
            imgproc::put_text(
                &mut total_processed,
                &fps_text,
                Point::new(text_x, text_y),
                imgproc::FONT_HERSHEY_SIMPLEX,
                2.0,
                Scalar::new(255.0, 255.0, 255.0, 255.0),
                2,
                imgproc::LINE_8,
                false,
            )?;

            let mut sliding_texted = sliding_with_line.clone();
            imgproc::put_text(
                &mut sliding_texted,
                &angle_text,
                Point::new(20, 100),
                imgproc::FONT_HERSHEY_SIMPLEX,
                2.0,
                Scalar::new(255.0, 255.0, 255.0, 255.0),
                2,
                imgproc::LINE_8,
                false,
            )?;

            // 최종 합성 결과 (좌: 슬라이딩윈도우, 우: 최종)
            let mut merged = Mat::default();
            hconcat_2(&sliding_texted, &total_processed, &mut merged)?;
            Ok(merged)
        } else {
            Ok(Mat::default())
        }
    }

    /// 차선 검출을 시작하는 함수입니다.
    /// `mode` 값에 따라 웹캠(0번 카메라) 혹은 파일(`"./video/challenge.mp4"`)을 열어 차선 검출 루프를 수행합니다.
    ///
    /// 'q' 키를 누르면 종료합니다.
    ///
    /// # 인자
    /// * `mode` - `CAM_MODE`와 같으면 웹캠을, 그렇지 않으면 지정된 동영상을 이용
    pub fn start_detection(&mut self, mode: i32, visible: bool) -> Result<()> {
        // 확인용 영상 출력 여부
        self.visible = visible;

        // 캡처 소스 준비 (웹캠 or 파일)
        let mut cap = if mode == CAM_MODE {
            // 웹캠
            videoio::VideoCapture::new(0, videoio::CAP_ANY)?
        } else {
            // 동영상 파일
            videoio::VideoCapture::from_file("./video/challenge.mp4", videoio::CAP_ANY)?
        };

        if self.visible == true {
            // 결과를 보여줄 윈도우 생성
            highgui::named_window("CAM View", highgui::WINDOW_AUTOSIZE)?;
        }

        loop {
            let mut frame = Mat::default();
            match cap.read(&mut frame) {
                Ok(is_read) => {
                    if !is_read || frame.empty() {
                        // 더 이상 프레임이 없거나 읽기에 실패
                        break;
                    }
                }
                Err(e) => {
                    eprintln!("Failed to read frame: {:?}", e);
                    break;
                }
            }

            // (1) 해상도를 1280x720으로 리사이즈
            let mut resized = Mat::default();
            imgproc::resize(
                &frame,
                &mut resized,
                core::Size::new(1280, 720),
                0.0,
                0.0,
                imgproc::INTER_LINEAR,
            )?;

            let start_time = Instant::now();
            // (2) 처리 파이프라인 실행
            let merged = self.processing(&resized)?;

            if self.visible == true {
                // (3) 결과 영상 출력
                highgui::imshow("CAM View", &merged)?;

                // (4) 'q' 키 입력 시 종료
                let key = highgui::wait_key(1)?;
                if key == 113 {
                    // 'q' 키(ASCII)
                    self.exit_flag = true;
                }

                if self.exit_flag {
                    break;
                }
            }

            // FPS 출력 (콘솔)
            let elapsed = start_time.elapsed().as_secs_f32();
            let fps = 1.0 / elapsed;
            println!("{}", fps);
        }

        Ok(())
    }
}
/// (x, y) 데이터로부터 1차 다항식(직선: y = a*x + b)을 최소제곱법(OLS)으로 피팅하는 함수.
///
/// # 인자
/// * `xs` - x좌표 배열
/// * `ys` - y좌표 배열
///
/// # 반환
/// * [a, b]: a=기울기, b=절편. (None: 점이 부족하거나 기울기 계산 불가능 시)
///
/// # Rust 문법 장점
/// - `Option<[f64; 2]>`로 성공/실패 여부를 명시적으로 전달 가능합니다.
///   C++의 경우 보통 예외나 특정 flag 값을 리턴하지만, Rust는 `Option` enum으로
///   실패 시 None을 리턴해, 호출부에서 안전하게 처리 가능.
#[inline(always)]
fn polyfit_1d(xs: &[f64], ys: &[f64]) -> Option<[f64; 2]> {
    if xs.len() < 2 || xs.len() != ys.len() {
        return None;
    }

    let n = xs.len() as f64;
    let sum_x = xs.iter().sum::<f64>();
    let sum_y = ys.iter().sum::<f64>();
    let sum_x2 = xs.iter().map(|&x| x * x).sum::<f64>();
    let sum_xy = xs.iter().zip(ys.iter()).map(|(&x, &y)| x * y).sum::<f64>();

    let denom = n * sum_x2 - sum_x * sum_x;
    if denom.abs() < 1e-12 {
        return None;
    }

    let a = (n * sum_xy - sum_x * sum_y) / denom;
    let b = (sum_y - a * sum_x) / n;

    Some([a, b])
}

/// 2차 다항식 y = ax^2 + bx + c 를 최소제곱법(OLS)으로 피팅.
///
/// # 인자
/// * `xs`, `ys`: 데이터 점들의 x좌표, y좌표
///
/// # 반환
/// * [a, b, c] 혹은 None(계산 불가)
///
/// # Rust 문법 설명
/// - `Some([a_, b_, c_])` 형태로 배열을 직접 반환할 수 있습니다.
///   C/C++이라면 구조체나 포인터를 써야 하는데, Rust에서는 튜플/배열/구조체 등 다양한
///   방법으로 반환 형식을 명확히 표현할 수 있습니다.
#[inline(always)]
fn polyfit_2d(xs: &[f64], ys: &[f64]) -> Option<[f64; 3]> {
    if xs.len() < 3 || xs.len() != ys.len() {
        return None;
    }

    let n = xs.len();
    let mut x2_sum = 0.0;
    let mut x_sum = 0.0;
    let mut one_sum = 0.0;
    let mut x3_sum = 0.0;
    let mut x4_sum = 0.0;
    let mut xy_sum = 0.0;
    let mut x2y_sum = 0.0;
    let mut y_sum = 0.0;

    for i in 0..n {
        let x = xs[i];
        let y = ys[i];
        x2_sum += x * x;
        x_sum += x;
        one_sum += 1.0;
        x3_sum += x * x * x;
        x4_sum += x * x * x * x;
        xy_sum += x * y;
        x2y_sum += x * x * y;
        y_sum += y;
    }

    // Normal Equation
    let a11 = x4_sum;
    let a12 = x3_sum;
    let a13 = x2_sum;

    let a21 = x3_sum;
    let a22 = x2_sum;
    let a23 = x_sum;

    let a31 = x2_sum;
    let a32 = x_sum;
    let a33 = one_sum;

    let b1 = x2y_sum;
    let b2 = xy_sum;
    let b3 = y_sum;

    // 3x3 행렬식
    let det = a11 * (a22 * a33 - a23 * a32) - a12 * (a21 * a33 - a23 * a31)
        + a13 * (a21 * a32 - a22 * a31);

    if det.abs() < 1e-12 {
        return None;
    }

    // Cramer's rule
    let det_a = |a11_, a12_, a13_, a21_, a22_, a23_, a31_, a32_, a33_| {
        a11_ * (a22_ * a33_ - a23_ * a32_) - a12_ * (a21_ * a33_ - a23_ * a31_)
            + a13_ * (a21_ * a32_ - a22_ * a31_)
    };

    let det0 = det_a(b1, a12, a13, b2, a22, a23, b3, a32, a33);
    let det1 = det_a(a11, b1, a13, a21, b2, a23, a31, b3, a33);
    let det2 = det_a(a11, a12, b1, a21, a22, b2, a31, a32, b3);

    let a_ = det0 / det;
    let b_ = det1 / det;
    let c_ = det2 / det;

    Some([a_, b_, c_])
}

/// 벡터의 마지막 10개 원소의 평균값을 구합니다.
/// 길이가 10보다 작으면 전체 평균을 구합니다.
#[inline(always)]
fn mean_of_last_10(vec: &Vec<f64>) -> f64 {
    let len = vec.len();
    if len == 0 {
        return 0.0;
    }
    let start = len.saturating_sub(10);
    let slice = &vec[start..];
    let sum: f64 = slice.iter().sum();
    sum / (slice.len() as f64)
}

/// 두 Mat을 가로로 이어붙여 `dst`에 저장합니다.
///
/// # 인자
/// * `img1`, `img2`: 이어붙일 영상
/// * `dst`: 결과를 담을 Mat
fn hconcat_2(img1: &Mat, img2: &Mat, dst: &mut Mat) -> Result<()> {
    let mut srcs: Vector<Mat> = Vector::new();
    srcs.push(img1.clone());
    srcs.push(img2.clone());
    core::hconcat(&srcs, dst)?;
    Ok(())
}

/// **내부 헬퍼 함수 (병렬 처리 적용)**
/// 이진 영상에서 각 row별로 0이 아닌 픽셀의 x좌표를 모아둔 벡터를 생성합니다.
/// Rayon을 이용해 각 행(row)의 연산을 여러 CPU 코어에 분배하여 처리 속도를 높입니다.
///
/// # 반환
/// * `Vec<Vec<i32>>` : `i`번째 원소는 `i`번째 row의 nonzero 픽셀 x좌표 목록
fn get_nonzero_points_by_row(binary_img: &Mat) -> Result<Vec<Vec<i32>>> {
    // (1) 단일 채널(CV_8UC1) 영상인지 확인
    if binary_img.channels() != 1 {
        return Err(opencv::Error::new(
            opencv::core::StsUnmatchedFormats,
            "Expected a single-channel (CV_8UC1) image.",
        ));
    }

    let rows = binary_img.rows();
    let cols = binary_img.cols();

    // (2) 한 행(row)이 차지하는 바이트 (stride) 구하기
    let step = binary_img.step1(0)? as usize;

    // (3) data_bytes() → Mat 전체 버퍼를 &[u8] 형태로 가져옴
    let data = binary_img.data_bytes()?;

    // (4) 각 row를 병렬로 순회하며, nonzero 픽셀(x좌표)만 수집
    // 기존 for 루프를 Rayon의 into_par_iter()로 변경하여 병렬 처리
    let result: Vec<Vec<i32>> = (0..rows)
        .into_par_iter() // 이 부분이 핵심입니다. 반복 작업을 병렬로 처리하도록 만듭니다.
        .map(|y| {
            let row_start = y as usize * step;
            let row_end = row_start + cols as usize;
            let row_slice = &data[row_start..row_end.min(data.len())];

            let mut row_vec = Vec::with_capacity(cols as usize / 4); // 약간의 사전 할당
            for x in 0..cols {
                if row_slice[x as usize] != 0 {
                    row_vec.push(x);
                }
            }
            row_vec
        })
        .collect(); // 각 스레드에서 처리된 결과를 하나의 Vec으로 모읍니다.

    Ok(result)
}

#[derive(Clone, Copy, Debug)]
pub struct LaneTaskConfig {
    pub use_kalman: bool,
}

impl LaneTaskConfig {
    pub fn new(use_kalman: bool) -> Self {
        Self { use_kalman }
    }
}
