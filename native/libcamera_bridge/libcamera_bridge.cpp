#include <libcamera/base/span.h>
#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include <algorithm>
#include <cstdio>
#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include <sys/mman.h>
#include <system_error>

namespace {

class CameraBridge {
public:
    static std::unique_ptr<CameraBridge> create(uint32_t width,
                                                uint32_t height,
                                                uint32_t fps,
                                                std::string &err);

    ~CameraBridge();

    int capture(uint8_t *buffer,
                size_t buffer_len,
                size_t *out_size,
                uint64_t *timestamp_ns);

    uint32_t stride() const { return stride_; }
    uint32_t bytes_per_pixel() const { return bytes_per_pixel_; }
    uint32_t width() const { return width_; }
    uint32_t height() const { return height_; }

private:
    CameraBridge() = default;

    struct PlaneMapping {
        void *addr = nullptr;
        size_t length = 0;
    };

    struct BufferContext {
        libcamera::FrameBuffer *buffer;
        std::vector<PlaneMapping> planes;
    };

    bool init(uint32_t width, uint32_t height, uint32_t fps, std::string &err);
    void shutdown();
    void requestComplete(libcamera::Request *request);
    void unmapBuffers();

    std::unique_ptr<libcamera::CameraManager> manager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;
    libcamera::Stream *stream_ = nullptr;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    std::vector<BufferContext> buffers_;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;

    std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<libcamera::Request *> completed_;
    bool running_ = false;

    uint32_t width_ = 0;
    uint32_t height_ = 0;
    uint32_t stride_ = 0;
    uint32_t bytes_per_pixel_ = 0;
};

std::unique_ptr<CameraBridge> CameraBridge::create(uint32_t width,
                                                   uint32_t height,
                                                   uint32_t fps,
                                                   std::string &err)
{
    std::unique_ptr<CameraBridge> bridge(new CameraBridge());
    if (!bridge->init(width, height, fps, err)) {
        bridge->shutdown();
        return nullptr;
    }
    return bridge;
}

bool CameraBridge::init(uint32_t width,
                        uint32_t height,
                        uint32_t fps,
                        std::string &err)
{
    manager_ = std::make_unique<libcamera::CameraManager>();
    if (manager_->start()) {
        err = "Failed to start libcamera manager";
        return false;
    }

    if (manager_->cameras().empty()) {
        err = "No libcamera compatible cameras found";
        return false;
    }

    camera_ = manager_->cameras()[0];
    if (!camera_) {
        err = "Unable to acquire default camera";
        return false;
    }

    if (camera_->acquire()) {
        err = "Failed to acquire camera instance";
        return false;
    }

    struct Candidate {
        uint32_t width;
        uint32_t height;
        libcamera::PixelFormat format;
        const char *label;
    };

    const std::vector<Candidate> candidates = {
        {width, height, libcamera::formats::BGR888, "requested BGR888"},
        {width, height, libcamera::formats::XRGB8888, "requested XRGB8888"},
        {1296, 972, libcamera::formats::BGR888, "fallback 1296x972 BGR888"},
        {1296, 972, libcamera::formats::XRGB8888, "fallback 1296x972 XRGB8888"},
        {640, 480, libcamera::formats::BGR888, "fallback 640x480 BGR888"},
        {640, 480, libcamera::formats::XRGB8888, "fallback 640x480 XRGB8888"},
    };

    std::vector<libcamera::StreamRole> roles{libcamera::StreamRole::Viewfinder};

    for (const auto &candidate : candidates) {
        std::fprintf(stderr,
                     "[libcamera_bridge] trying candidate: %s (%ux%u, fmt=%s)\n",
                     candidate.label,
                     candidate.width,
                     candidate.height,
                     candidate.format.toString().c_str());

        auto attempt_config = camera_->generateConfiguration(roles);
        if (!attempt_config) {
            std::fprintf(stderr, "[libcamera_bridge] generateConfiguration failed\n");
            continue;
        }

        libcamera::StreamConfiguration &attempt_stream_cfg = attempt_config->at(0);
        attempt_stream_cfg.size.width = candidate.width;
        attempt_stream_cfg.size.height = candidate.height;
        attempt_stream_cfg.pixelFormat = candidate.format;
        attempt_stream_cfg.bufferCount = std::max(attempt_stream_cfg.bufferCount, 2U);

        auto status = attempt_config->validate();
        if (status == libcamera::CameraConfiguration::Invalid) {
            std::fprintf(stderr, "[libcamera_bridge] validate returned Invalid\n");
            continue;
        }

        std::fprintf(stderr,
                     "[libcamera_bridge] validated stream: %s\n",
                     attempt_stream_cfg.toString().c_str());

        if (camera_->configure(attempt_config.get())) {
            std::fprintf(stderr, "[libcamera_bridge] configure failed\n");
            continue;
        }

        libcamera::Stream *attempt_stream = attempt_stream_cfg.stream();
        if (!attempt_stream) {
            std::fprintf(stderr, "[libcamera_bridge] stream() returned nullptr\n");
            continue;
        }

        auto attempt_allocator = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
        int alloc_rc = attempt_allocator->allocate(attempt_stream);
        if (alloc_rc < 0) {
            std::fprintf(stderr,
                         "[libcamera_bridge] allocate failed rc=%d\n",
                         alloc_rc);
            continue;
        }
        std::fprintf(stderr,
                     "[libcamera_bridge] allocate succeeded buffers=%d\n",
                     alloc_rc);

        const auto &frame_buffers = attempt_allocator->buffers(attempt_stream);
        if (frame_buffers.empty()) {
            std::fprintf(stderr, "[libcamera_bridge] allocator returned no buffers\n");
            continue;
        }

        std::vector<BufferContext> attempt_buffers;
        attempt_buffers.reserve(frame_buffers.size());
        std::vector<std::unique_ptr<libcamera::Request>> attempt_requests;
        attempt_requests.reserve(frame_buffers.size());

        bool mapping_ok = true;

        for (const auto &buffer : frame_buffers) {
            std::vector<PlaneMapping> plane_mappings;
            plane_mappings.reserve(buffer->planes().size());
            for (const libcamera::FrameBuffer::Plane &plane : buffer->planes()) {
                if (!plane.fd.isValid()) {
                    std::fprintf(stderr, "[libcamera_bridge] plane fd invalid\n");
                    mapping_ok = false;
                    break;
                }

                void *addr = mmap(nullptr,
                                  plane.length,
                                  PROT_READ,
                                  MAP_SHARED,
                                  plane.fd.get(),
                                  static_cast<off_t>(plane.offset));
                if (addr == MAP_FAILED) {
                    std::fprintf(stderr, "[libcamera_bridge] mmap failed\n");
                    mapping_ok = false;
                    break;
                }

                plane_mappings.push_back(PlaneMapping{addr, plane.length});
            }

            if (!mapping_ok) {
                for (const PlaneMapping &mapped_plane : plane_mappings) {
                    if (mapped_plane.addr) {
                        munmap(mapped_plane.addr, mapped_plane.length);
                    }
                }
                break;
            }

            std::unique_ptr<libcamera::Request> request = camera_->createRequest();
            if (!request) {
                std::fprintf(stderr, "[libcamera_bridge] createRequest failed\n");
                mapping_ok = false;
                break;
            }

            libcamera::Request *req = request.get();
            if (req->addBuffer(attempt_stream, buffer.get())) {
                std::fprintf(stderr, "[libcamera_bridge] addBuffer failed\n");
                mapping_ok = false;
                break;
            }

            attempt_buffers.push_back(BufferContext{buffer.get(), std::move(plane_mappings)});
            attempt_requests.push_back(std::move(request));
        }

        if (!mapping_ok) {
            for (BufferContext &ctx : attempt_buffers) {
                for (PlaneMapping &plane : ctx.planes) {
                    if (plane.addr) {
                        munmap(plane.addr, plane.length);
                    }
                }
            }
            continue;
        }

        if (camera_->start()) {
            std::fprintf(stderr, "[libcamera_bridge] camera start failed\n");
            for (BufferContext &ctx : attempt_buffers) {
                for (PlaneMapping &plane : ctx.planes) {
                    if (plane.addr) {
                        munmap(plane.addr, plane.length);
                    }
                }
            }
            continue;
        }

        bool queue_ok = true;
        for (auto &request : attempt_requests) {
            if (camera_->queueRequest(request.get())) {
                std::fprintf(stderr, "[libcamera_bridge] queueRequest failed\n");
                queue_ok = false;
                break;
            }
        }

        if (!queue_ok) {
            camera_->stop();
            for (BufferContext &ctx : attempt_buffers) {
                for (PlaneMapping &plane : ctx.planes) {
                    if (plane.addr) {
                        munmap(plane.addr, plane.length);
                    }
                }
            }
            continue;
        }

        allocator_ = std::move(attempt_allocator);
        config_ = std::move(attempt_config);
        stream_ = attempt_stream;
        buffers_ = std::move(attempt_buffers);
        requests_ = std::move(attempt_requests);

        camera_->requestCompleted.connect(this, &CameraBridge::requestComplete);

        running_ = true;
        width_ = config_->at(0).size.width;
        height_ = config_->at(0).size.height;
        stride_ = config_->at(0).stride;

        if (config_->at(0).pixelFormat == libcamera::formats::BGR888 ||
            config_->at(0).pixelFormat == libcamera::formats::RGB888) {
            bytes_per_pixel_ = 3;
        } else if (config_->at(0).pixelFormat == libcamera::formats::XRGB8888 ||
                   config_->at(0).pixelFormat == libcamera::formats::XBGR8888 ||
                   config_->at(0).pixelFormat == libcamera::formats::ARGB8888 ||
                   config_->at(0).pixelFormat == libcamera::formats::ABGR8888) {
            bytes_per_pixel_ = 4;
        } else if (width_ > 0 && stride_ >= width_) {
            bytes_per_pixel_ = std::max<uint32_t>(1U, stride_ / width_);
        } else {
            bytes_per_pixel_ = 3;
        }

        if (stride_ == 0) {
            stride_ = width_ * bytes_per_pixel_;
        }

        std::fprintf(stderr,
                     "[libcamera_bridge] streaming with %ux%u stride=%u bpp=%u pixelFormat=%s\n",
                     width_,
                     height_,
                     stride_,
                     bytes_per_pixel_,
                     config_->at(0).pixelFormat.toString().c_str());

        return true;
    }

    err = "Failed to configure any libcamera stream";
    return false;
}

void CameraBridge::shutdown()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        while (!completed_.empty()) {
            completed_.pop();
        }
    }
    cv_.notify_all();

    if (camera_) {
        camera_->requestCompleted.disconnect(this, &CameraBridge::requestComplete);
        camera_->stop();
        camera_->release();
        camera_.reset();
    }

    allocator_.reset();
    config_.reset();

    unmapBuffers();

    if (manager_) {
        manager_->stop();
        manager_.reset();
    }

    buffers_.clear();
    requests_.clear();
}

void CameraBridge::unmapBuffers()
{
    for (BufferContext &ctx : buffers_) {
        for (PlaneMapping &plane : ctx.planes) {
            if (plane.addr && plane.length) {
                munmap(plane.addr, plane.length);
                plane.addr = nullptr;
                plane.length = 0;
            }
        }
    }
}

CameraBridge::~CameraBridge()
{
    shutdown();
}

void CameraBridge::requestComplete(libcamera::Request *request)
{
    if (request->status() == libcamera::Request::RequestCancelled) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        completed_.push(request);
    }
    cv_.notify_one();
}

int CameraBridge::capture(uint8_t *buffer,
                          size_t buffer_len,
                          size_t *out_size,
                          uint64_t *timestamp_ns)
{
    libcamera::Request *request = nullptr;
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [&] { return !completed_.empty() || !running_; });
        if (!running_) {
            return -1;
        }
        request = completed_.front();
        completed_.pop();
    }

    if (!request) {
        return -2;
    }

    auto it = request->buffers().find(stream_);
    if (it == request->buffers().end()) {
        return -3;
    }

    libcamera::FrameBuffer *frame_buffer = it->second;
    auto buffer_it = std::find_if(buffers_.begin(), buffers_.end(),
                                  [&](const BufferContext &ctx) { return ctx.buffer == frame_buffer; });
    if (buffer_it == buffers_.end()) {
        return -4;
    }

    if (buffer_it->planes.empty()) {
        return -5;
    }

    const PlaneMapping &plane = buffer_it->planes[0];
    size_t plane_size = plane.length;
    size_t copy_size = std::min(plane_size, buffer_len);

    std::memcpy(buffer, plane.addr, copy_size);

    if (out_size) {
        *out_size = copy_size;
    }

    if (timestamp_ns) {
        *timestamp_ns = 0;
    }

    request->reuse(libcamera::Request::ReuseBuffers);
    if (camera_->queueRequest(request)) {
        return -6;
    }

    return 0;
}

} // namespace

extern "C" {

struct LibcameraBridgeOpaque {
    std::unique_ptr<CameraBridge> inner;
};

static void set_error(char *err_buf, size_t err_len, const std::string &err)
{
    if (!err_buf || err_len == 0) {
        return;
    }
    std::strncpy(err_buf, err.c_str(), err_len - 1);
    err_buf[err_len - 1] = '\0';
}

LibcameraBridgeOpaque *libcamera_bridge_create(uint32_t width,
                                               uint32_t height,
                                               uint32_t fps,
                                               uint32_t *out_stride,
                                               uint32_t *out_bpp,
                                               char *err_buf,
                                               size_t err_len)
{
    std::string err;
    std::unique_ptr<CameraBridge> bridge = CameraBridge::create(width, height, fps, err);
    if (!bridge) {
        set_error(err_buf, err_len, err);
        return nullptr;
    }

    if (out_stride) {
        *out_stride = bridge->stride();
    }

    if (out_bpp) {
        *out_bpp = bridge->bytes_per_pixel();
    }

    LibcameraBridgeOpaque *opaque = new LibcameraBridgeOpaque();
    opaque->inner = std::move(bridge);
    return opaque;
}

int libcamera_bridge_capture(LibcameraBridgeOpaque *opaque,
                             uint8_t *buffer,
                             size_t buffer_len,
                             size_t *out_size,
                             uint64_t *timestamp_ns,
                             char *err_buf,
                             size_t err_len)
{
    if (!opaque || !opaque->inner) {
        set_error(err_buf, err_len, "Invalid libcamera bridge handle");
        return -100;
    }

    int rc = opaque->inner->capture(buffer, buffer_len, out_size, timestamp_ns);
    if (rc != 0) {
        set_error(err_buf, err_len, "libcamera capture failed");
    }
    return rc;
}

void libcamera_bridge_destroy(LibcameraBridgeOpaque *opaque)
{
    if (!opaque) {
        return;
    }
    delete opaque;
}

} // extern "C"
