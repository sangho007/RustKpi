use std::collections::HashSet;
use std::env;
use std::path::{Path, PathBuf};

fn main() {
    let mut build = cc::Build::new();
    let bridge_file = "src/bsw/lib/libcamera_bridge.cpp";

    let mut include_paths: Vec<PathBuf> = Vec::new();
    let mut link_libs: Vec<String> = Vec::new();
    let mut link_paths: Vec<PathBuf> = Vec::new();

    if let Ok(lib) = pkg_config::Config::new()
        .cargo_metadata(true)
        .probe("libcamera")
    {
        include_paths.extend(lib.include_paths.iter().cloned());
        link_libs.extend(lib.libs.iter().cloned());
        link_paths.extend(lib.link_paths.iter().cloned());
    }

    // Allow overriding C++ standard flags via the environment.
    build
        .cpp(true)
        .file(bridge_file)
        .flag_if_supported("-std=c++17")
        .flag_if_supported("-fPIC");

    // Honour additional include paths from pkg-config.
    let mut seen = HashSet::new();
    let mut add_include = |path: PathBuf| {
        if seen.insert(path.clone()) {
            build.include(path);
        }
    };

    for include_path in include_paths.iter() {
        add_include(include_path.clone());
        if let Some(parent) = include_path.parent() {
            add_include(parent.to_path_buf());
        }
        add_include(include_path.join("libcamera"));
    }

    // Common fallback locations (Debian/Raspberry Pi packages).
    const FALLBACKS: &[&str] = &[
        "/usr/include",
        "/usr/include/libcamera",
        "/usr/include/aarch64-linux-gnu",
        "/usr/include/aarch64-linux-gnu/libcamera",
        "/usr/local/include",
        "/usr/local/include/libcamera",
    ];

    for fallback in FALLBACKS {
        let path = Path::new(fallback);
        if path.exists() {
            add_include(path.to_path_buf());
        }
    }

    // Honour pkg-config link search paths and libraries if available.
    for path in link_paths {
        println!("cargo:rustc-link-search=native={}", path.display());
    }

    let mut saw_camera = false;
    let mut saw_camera_base = false;

    for lib in &link_libs {
        if lib == "camera" {
            saw_camera = true;
        } else if lib == "camera-base" {
            saw_camera_base = true;
        }
    }

    for lib in link_libs {
        println!("cargo:rustc-link-lib={}", lib);
    }

    if saw_camera || saw_camera_base {
        println!("cargo:rustc-link-arg=-Wl,--push-state,--no-as-needed");
        println!("cargo:rustc-link-arg=-Wl,--start-group");
        if saw_camera {
            println!("cargo:rustc-link-arg=-lcamera");
        }
        if saw_camera_base {
            println!("cargo:rustc-link-arg=-lcamera-base");
        }
        println!("cargo:rustc-link-arg=-Wl,--end-group");
        println!("cargo:rustc-link-arg=-Wl,--pop-state");
    }

    // Allow downstream customization.
    if let Ok(extra_flags) = env::var("LIBCAMERA_BRIDGE_CXXFLAGS") {
        for flag in extra_flags.split_whitespace() {
            build.flag(flag);
        }
    }

    println!("cargo:rerun-if-env-changed=LIBCAMERA_BRIDGE_CXXFLAGS");
    println!("cargo:rerun-if-changed={}", bridge_file);

    build.compile("libcamera_bridge");
}
