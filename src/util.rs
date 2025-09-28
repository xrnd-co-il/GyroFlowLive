// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2021-2022 Adrian <adrian.eddy at gmail>

use cpp::*;
use qmetaobject::*;
use std::ffi::c_void;

pub fn serde_json_to_qt_array(v: &serde_json::Value) -> QJsonArray {
    let mut ret = QJsonArray::default();
    if let Some(arr) = v.as_array() {
        for param in arr {
            match param {
                serde_json::Value::Number(v) => { ret.push(QJsonValue::from(v.as_f64().unwrap())); },
                serde_json::Value::Bool(v) => { ret.push(QJsonValue::from(*v)); },
                serde_json::Value::String(v) => { ret.push(QJsonValue::from(QString::from(v.clone()))); },
                serde_json::Value::Array(v) => { ret.push(QJsonValue::from(serde_json_to_qt_array(&serde_json::Value::Array(v.to_vec())))); },
                serde_json::Value::Object(_) => { ret.push(QJsonValue::from(serde_json_to_qt_object(param))); },
                serde_json::Value::Null => { /* ::log::warn!("null unimplemented");*/ }
            };
        }
    }
    ret
}
pub fn serde_json_to_qt_object(v: &serde_json::Value) -> QJsonObject {
    let mut map = QJsonObject::default();
    if let Some(obj) = v.as_object() {
        for (k, v) in obj {
            match v {
                serde_json::Value::Number(v) => { map.insert(k, QJsonValue::from(v.as_f64().unwrap())); },
                serde_json::Value::Bool(v) => { map.insert(k, QJsonValue::from(*v)); },
                serde_json::Value::String(v) => { map.insert(k, QJsonValue::from(QString::from(v.clone()))); },
                serde_json::Value::Array(v) => { map.insert(k, QJsonValue::from(serde_json_to_qt_array(&serde_json::Value::Array(v.to_vec())))); },
                serde_json::Value::Object(_) => { map.insert(k, QJsonValue::from(serde_json_to_qt_object(v))); },
                serde_json::Value::Null => { /* ::log::warn!("null unimplemented");*/ }
            };
        }
    }
    map
}

pub fn is_opengl() -> bool {
    cpp!(unsafe [] -> bool as "bool" {
        return QQuickWindow::graphicsApi() == QSGRendererInterface::OpenGLRhi;
    })
}

pub fn qt_queued_callback<T: QObject + 'static, T2: Send, F: FnMut(&T, T2) + 'static>(qobj: &T, mut cb: F) -> impl Fn(T2) + Send + Sync + Clone {
    let qptr = QPointer::from(qobj);
    qmetaobject::queued_callback(move |arg| {
        if let Some(this) = qptr.as_pinned() {
            let this = this.borrow();
            cb(this, arg);
        }
    })
}
pub fn qt_queued_callback_mut<T: QObject + 'static, T2: Send, F: FnMut(&mut T, T2) + 'static>(qobj: &T, mut cb: F) -> impl Fn(T2) + Send + Sync + Clone {
    let qptr = QPointer::from(qobj);
    qmetaobject::queued_callback(move |arg| {
        if let Some(this) = qptr.as_pinned() {
            let mut this = this.borrow_mut();
            cb(&mut this, arg);
        }
    })
}

#[macro_export]
macro_rules! wrap_simple_method {
    ($name:ident, $($param:ident:$type:ty),*) => {
        fn $name(&self, $($param:$type,)*) {
            self.stabilizer.$name($($param,)*);
        }
    };
    ($name:ident, $($param:ident:$type:ty),*; recompute) => {
        fn $name(&self, $($param:$type,)*) {
            self.stabilizer.$name($($param,)*);
            self.request_recompute();
        }
    };
    ($name:ident, $($param:ident:$type:ty),*; recompute$(; $extra_call:ident)*) => {
        fn $name(&mut self, $($param:$type,)*) {
            self.stabilizer.$name($($param,)*);
            self.request_recompute();
            $( self.$extra_call(); )*
        }
    };
}

cpp! {{
    #ifdef Q_OS_ANDROID
    #   include <QJniObject>
    #endif
    #include <QDesktopServices>
    #include <QStandardPaths>
    #include <QBuffer>
    #include <QImage>
    #include <QGuiApplication>
    #include <QObject>
    #include <QClipboard>
    #include <QEvent>
    #if (__APPLE__ + 0) || (__linux__ + 0)
    #   include <sys/resource.h>
    #endif

    static QObject *globalUrlCatcherPtr = nullptr;
    static QString pendingUrl;

    class QtEventFilter : public QObject {
    public:
        QtEventFilter(std::function<void(QUrl)> cb) : m_cb(cb) { }
        bool eventFilter(QObject *obj, QEvent *event) override {
            if (event->type() == QEvent::FileOpen) {
                m_cb(static_cast<QFileOpenEvent *>(event)->url());
            }
            return QObject::eventFilter(obj, event);
        }
        std::function<void(QUrl)> m_cb;
    };
}}
#[cfg(target_os = "android")]
#[allow(non_snake_case)]
#[no_mangle]
pub extern "system" fn Java_xyz_gyroflow_MainActivity_urlReceived(_vm: *mut c_void, _: *mut c_void, jstr: *mut c_void) {
    cpp!(unsafe [jstr as "void*"] {
        #ifdef Q_OS_ANDROID
            QString str = QJniObject((jstring)jstr).toString();
            if (globalUrlCatcherPtr) {
                QMetaObject::invokeMethod(globalUrlCatcherPtr, "catch_url_open", Qt::QueuedConnection, Q_ARG(QUrl, QUrl(str)));
            } else {
                pendingUrl = str;
            }
        #else
            (void)jstr;
        #endif
    });
}
pub fn set_url_catcher(ctlptr: *mut c_void) {
    cpp!(unsafe [ctlptr as "QObject *"] {
        globalUrlCatcherPtr = ctlptr;
        if (!pendingUrl.isEmpty()) {
            QMetaObject::invokeMethod(globalUrlCatcherPtr, "catch_url_open", Qt::QueuedConnection, Q_ARG(QUrl, QUrl(pendingUrl)));
            pendingUrl.clear();
        }
    });
}
pub fn register_url_handlers() {
    cpp!(unsafe [] {
        #if defined(Q_OS_ANDROID) || defined(Q_OS_IOS)
            QDesktopServices::setUrlHandler("content", globalUrlCatcherPtr, "catch_url_open");
            QDesktopServices::setUrlHandler("file",    globalUrlCatcherPtr, "catch_url_open");
        #endif
    });
}
pub fn unregister_url_handlers() {
    cpp!(unsafe [] {
        #if defined(Q_OS_ANDROID) || defined(Q_OS_IOS)
            QDesktopServices::unsetUrlHandler("content");
            QDesktopServices::unsetUrlHandler("file");
        #endif
    });
}
pub fn dispatch_url_event(url: QUrl) {
    cpp!(unsafe [url as "QUrl"] {
        QFileOpenEvent evt(url);
        qGuiApp->sendEvent(qGuiApp, &evt);
    });
}
pub fn qurl_to_encoded(url: QUrl) -> String {
    cpp!(unsafe [url as "QUrl"] -> QString as "QString" {
        return QString(url.toEncoded());
    }).to_string()
}
pub fn catch_qt_file_open<F: FnMut(QUrl)>(cb: F) {
    let func: Box<dyn FnMut(QUrl)> = Box::new(cb);
    let cb_ptr = Box::into_raw(func);
    cpp!(unsafe [cb_ptr as "TraitObject2"] {
        qGuiApp->installEventFilter(new QtEventFilter([cb_ptr](QUrl url) {
            rust!(Rust_catch_qt_file_open [cb_ptr: *mut dyn FnMut(QUrl) as "TraitObject2", url: QUrl as "QUrl"] {
                let mut cb = unsafe { Box::from_raw(cb_ptr) };
                cb(url.clone());
                let _ = Box::into_raw(cb); // leak again so it doesn't get deleted here
            });
        }));
    });
}

pub fn open_file_externally(url: QUrl) {
    unregister_url_handlers();
    cpp!(unsafe [url as "QUrl"] { QDesktopServices::openUrl(url); });
    register_url_handlers();
}

pub fn get_data_location() -> String {
    cpp!(unsafe [] -> QString as "QString" {
        return QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    }).into()
}

pub fn update_rlimit() {
    cpp!(unsafe [] {
        #if (__APPLE__ + 0) || (__linux__ + 0)
            // Increase open file limit, because it gets hit pretty quickly with R3D or BRAW in render queue
            struct rlimit limit;
            if (::getrlimit(RLIMIT_NOFILE, &limit) == 0) {
                if (limit.rlim_cur < 4096) {
                    limit.rlim_cur = 4096;
                    if (limit.rlim_max < 4096)
                        limit.rlim_max = 4096;
                    if (::setrlimit(RLIMIT_NOFILE, &limit) != 0) {
                        qDebug() << "Failed to set RLIMIT_NOFILE to 4096!";
                    }
                }
            }
        #endif
    });
}

pub fn set_android_context() {
    #[cfg(target_os = "android")]
    {
        let jvm = cpp!(unsafe [] -> *mut c_void as "void *" {
            #ifdef Q_OS_ANDROID
                return QJniEnvironment::javaVM();
            #else
                return nullptr;
            #endif
        });
        let activity = cpp!(unsafe [] -> *mut c_void as "void *" {
            #ifdef Q_OS_ANDROID
                auto ctx = QNativeInterface::QAndroidApplication::context();
                return QJniEnvironment::getJniEnv()->NewGlobalRef(ctx.object());
            #else
                return nullptr;
            #endif
        });
        unsafe { ndk_context::initialize_android_context(jvm, activity); }
    }
}

pub fn init_logging() {
    use simplelog::*;

    let log_config = [ "mp4parse", "wgpu", "naga", "akaze", "ureq", "rustls", "mdk" ]
        .into_iter()
        .fold(ConfigBuilder::new(), |mut cfg, x| { cfg.add_filter_ignore_str(x); cfg })
        .build();
    let file_log_config = [ "mp4parse", "wgpu", "naga", "akaze", "ureq", "rustls" ]
        .into_iter()
        .fold(ConfigBuilder::new(), |mut cfg, x| { cfg.add_filter_ignore_str(x); cfg })
        .build();

    #[cfg(target_os = "android")]
    WriteLogger::init(LevelFilter::Debug, log_config, crate::util::AndroidLog::default()).unwrap();

    #[cfg(not(target_os = "android"))]
    {
        let exe_loc = gyroflow_core::settings::data_dir().join("gyroflow.log");
        if let Ok(file_log) = std::fs::File::create(exe_loc) {
            let _ = CombinedLogger::init(vec![
                TermLogger::new(LevelFilter::Debug, log_config, TerminalMode::Mixed, ColorChoice::Auto),
                WriteLogger::new(LevelFilter::Debug, file_log_config, file_log)
            ]);
        } else {
            let _ = TermLogger::init(LevelFilter::Debug, log_config, TerminalMode::Mixed, ColorChoice::Auto);
        }
    }

    qmetaobject::log::init_qt_to_rust();

    qml_video_rs::video_item::MDKVideoItem::setLogHandler(|level: i32, text: &str| {
        match level {
            1 => { ::log::error!(target: "mdk", "[MDK] {}", text.trim()); },
            2 => { ::log::warn!(target: "mdk", "[MDK] {}", text.trim()); },
            3 => { ::log::info!(target: "mdk", "[MDK] {}", text.trim()); },
            4 => { ::log::debug!(target: "mdk", "[MDK] {}", text.trim()); },
            _ => { }
        }
    });
}

pub fn install_crash_handler() -> std::io::Result<()> {
    let cur_dir = std::env::current_dir()?;

    #[cfg(not(any(target_os = "android", target_os = "ios")))]
    {
        let os_str = cur_dir.as_os_str();
        let path: Vec<breakpad_sys::PathChar> = {
            #[cfg(windows)]
            {
                use std::os::windows::ffi::OsStrExt;
                os_str.encode_wide().collect()
            }
            #[cfg(unix)]
            {
                use std::os::unix::ffi::OsStrExt;
                Vec::from(os_str.as_bytes())
            }
        };

        unsafe {
            extern "C" fn callback(path: *const breakpad_sys::PathChar, path_len: usize, _ctx: *mut c_void) {
                let path_slice = unsafe { std::slice::from_raw_parts(path, path_len) };

                let path = {
                    #[cfg(windows)]
                    {
                        use std::os::windows::ffi::OsStringExt;
                        std::path::PathBuf::from(std::ffi::OsString::from_wide(path_slice))
                    }
                    #[cfg(unix)]
                    {
                        use std::os::unix::ffi::OsStrExt;
                        std::path::PathBuf::from(std::ffi::OsStr::from_bytes(path_slice).to_owned())
                    }
                };

                println!("Crashdump written to {}", path.display());
            }

            breakpad_sys::attach_exception_handler(
                path.as_ptr(),
                path.len(),
                callback,
                std::ptr::null_mut(),
                breakpad_sys::INSTALL_BOTH_HANDLERS,
            );
        }
    }

    // Upload crash dumps
    crate::core::run_threaded(move || {
        if let Ok(files) = std::fs::read_dir(cur_dir) {
            for path in files.flatten() {
                let path = path.path();
                if path.to_string_lossy().ends_with(".dmp") {
                    if let Ok(content) = std::fs::read(&path) {
                        if let Ok(Ok(body)) = ureq::post("https://api.gyroflow.xyz/upload_dump").header("Content-Type", "application/octet-stream").send(&content).map(|x| x.into_body().read_to_string()) {
                            ::log::debug!("Minidump uploaded: {}", body.as_str());
                            let _ = std::fs::remove_file(path);
                        }
                    }
                }
            }
        }
    });
    Ok(())
}

#[cfg(target_os = "android")]
pub fn android_log(v: String) {
    use std::ffi::{CStr, CString};
    let tag = CStr::from_bytes_with_nul(b"Gyroflow\0").unwrap();
    if let Ok(msg) = CString::new(v) {
        unsafe {
            ndk_sys::__android_log_write(ndk_sys::android_LogPriority::ANDROID_LOG_DEBUG.0 as std::os::raw::c_int, tag.as_ptr(), msg.as_ptr());
        }
    }
}

#[cfg(target_os = "android")]
#[derive(Default)]
pub struct AndroidLog { buf: String }
#[cfg(target_os = "android")]
impl std::io::Write for AndroidLog {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        if let Ok(s) = String::from_utf8(buf.to_vec()) {
            self.buf.push_str(&s);
        };
        if self.buf.contains('\n') {
            self.flush()?;
        }
        Ok(buf.len())
    }
    fn flush(&mut self) -> std::io::Result<()> { android_log(self.buf.clone()); self.buf.clear(); Ok(()) }
}

pub fn tr(context: &str, text: &str) -> String {
    let context = QString::from(context);
    let text = QString::from(text);
    cpp!(unsafe [context as "QString", text as "QString"] -> QString as "QString" {
        return QCoreApplication::translate(qUtf8Printable(context), qUtf8Printable(text));
    }).to_string()
}

pub fn qt_graphics_api() -> QString {
    cpp!(unsafe [] -> QString as "QString" {
        switch (QQuickWindow::graphicsApi()) {
            case QSGRendererInterface::OpenGL:     return "opengl";
            case QSGRendererInterface::Direct3D11: return "directx";
            case QSGRendererInterface::Vulkan:     return "vulkan";
            case QSGRendererInterface::Metal:      return "metal";
            default: return "unknown";
        }
    })
}

pub fn get_version() -> String {
    let ver = env!("CARGO_PKG_VERSION");
    if option_env!("GITHUB_REF").map_or(false, |x| x.contains("tags")) {
        ver.to_string() // Official, tagged version
    } else if let Some(gh_run) = option_env!("GITHUB_RUN_NUMBER") {
        format!("{} (gh{})", ver, gh_run)
    } else if let Some(time) = option_env!("BUILD_TIME") {
        format!("{} (dev{})", ver, time)
    } else {
        ver.to_string()
    }
}
pub fn copy_to_clipboard(text: QString) {
    cpp!(unsafe [text as "QString"] { QGuiApplication::clipboard()->setText(text); })
}

pub fn save_exe_location() {
    if let Ok(exe_path) = std::env::current_exe() {
        if cfg!(target_os = "macos") {
            if let Some(parent) = exe_path.parent() { // MacOS
                if let Some(parent) = parent.parent() { // Contents
                    if let Some(parent) = parent.parent() { // Gyroflow.app
                        gyroflow_core::settings::set("exeLocation", parent.to_string_lossy().into());
                    }
                }
            }
        } else {
            #[allow(unused_mut)]
            let mut exe_str = exe_path.to_string_lossy().to_string();

            #[cfg(target_os = "windows")]
            if exe_str.contains("29160AdrianRoss.Gyroflow") {
                let parts = exe_str.split("\\").collect::<Vec<_>>();
                let parts = parts.into_iter().rev().skip(1).next().unwrap_or("").split("_").collect::<Vec<_>>();
                if let Some(publisher) = parts.first() {
                    if let Some(app_id) = parts.last() {
                        if !publisher.is_empty() && !app_id.is_empty() {
                            exe_str = format!("shell:AppsFolder\\{publisher}_{app_id}!Gyroflow");
                        }
                    }
                }
            }
            #[cfg(target_os = "linux")]
            if exe_str.contains("/tmp/.mount") {
                if let Ok(appimg) = std::env::var("APPIMAGE") {
                    if !appimg.is_empty() {
                        exe_str = appimg;
                    }
                }
            }

            gyroflow_core::settings::set("exeLocation", exe_str.into());
        }
    }
}

pub fn image_data_to_base64(w: u32, h: u32, s: u32, data: &[u8]) -> QString {
    let ptr = data.as_ptr();
    cpp!(unsafe [w as "uint32_t", h as "uint32_t", s as "uint32_t", ptr as "const uint8_t *"] -> QString as "QString" {
        QImage img(ptr, w, h, s, QImage::Format_RGBA8888_Premultiplied);
        QByteArray byteArray;
        QBuffer buffer(&byteArray);
        buffer.open(QIODevice::WriteOnly);
        img.save(&buffer, "JPEG", 50);
        QString b64("data:image/jpg;base64,");
        b64.append(QString::fromLatin1(byteArray.toBase64().data()));
        return b64;
    })
}

pub fn image_to_b64(img: QImage) -> QString {
    cpp!(unsafe [img as "QImage"] -> QString as "QString" {
        QByteArray byteArray;
        QBuffer buffer(&byteArray);
        buffer.open(QIODevice::WriteOnly);
        img.save(&buffer, "JPEG", 50);
        QString b64("data:image/jpg;base64,");
        b64.append(QString::fromLatin1(byteArray.toBase64().data()));
        return b64;
    })
}

pub fn update_file_times(output_url: &str, input_url: &str, additional_ms: Option<f64>) {
    if let Err(e) = || -> std::io::Result<()> {
        let input_path = gyroflow_core::filesystem::url_to_path(input_url);
        let output_path = gyroflow_core::filesystem::url_to_path(output_url);
        if input_path.is_empty() || output_path.is_empty() {
            return Err(std::io::Error::new(std::io::ErrorKind::PermissionDenied, format!("Can't get path from url! Input: {input_url} / {input_path}, output: {output_url} / {output_path}")));
        }
        let mut org_time_c = filetime_creation::FileTime::from_creation_time(&std::fs::metadata(&input_path)?);
        let mut org_time_m = filetime_creation::FileTime::from_last_modification_time(&std::fs::metadata(&input_path)?);
        if let Some(additional_ms) = additional_ms {
            if additional_ms > 0.0 {
                if let Some(ctime) = org_time_c {
                    org_time_c = Some(filetime_creation::FileTime::from_unix_time(ctime.unix_seconds() + (additional_ms / 1000.0).round() as i64, ctime.nanoseconds()));
                }
                org_time_m = filetime_creation::FileTime::from_unix_time(org_time_m.unix_seconds() + (additional_ms / 1000.0).round() as i64, org_time_m.nanoseconds());
            }
        }
        if cfg!(target_os = "windows") {
            if let Some(org_time_c) = org_time_c {
                ::log::debug!("Updating creation time of {} to {}", output_path, org_time_c.to_string());
                filetime_creation::set_file_ctime(output_path.clone(), org_time_c)?;
            }
        }
        ::log::debug!("Updating modification time of {} to {}", output_path, org_time_m.to_string());
        filetime_creation::set_file_mtime(output_path, org_time_m)?;

        Ok(())
    }() { ::log::warn!("Failed to update file times: {e:?}"); }
}

pub fn is_store_package() -> bool {
    #[cfg(target_os = "windows")]
    unsafe {
        let mut len = 0;
        let _ = windows::Win32::Storage::Packaging::Appx::GetCurrentPackageFullName(&mut len, windows::core::PWSTR::null());
        if len > 0 {
            return true;
        }
    }

    // Only the app from App Store is sandboxed on MacOS
    if cfg!(target_os = "macos") && gyroflow_core::filesystem::is_sandboxed() {
        return true;
    }

    false
}

pub fn is_insta360(input_url: &str) -> bool {
    use std::io::*;
    let mut buf = vec![0u8; 32];
    let base = gyroflow_core::filesystem::get_engine_base();
    if let Ok(mut input) = gyroflow_core::filesystem::open_file(&base, input_url, false, false) {
        let _ = input.seek(SeekFrom::End(-32));
        let _ = input.read_exact(&mut buf);
    }
    &buf == b"8db42d694ccc418790edff439fe026bf"
}
pub fn copy_insta360_metadata(output_url: &str, input_url: &str) -> Result<(), gyroflow_core::filesystem::FilesystemError> {
    use std::io::*;
    pub const HEADER_SIZE: usize = 32 + 4 + 4 + 32; // padding(32), size(4), version(4), magic(32)
    pub const MAGIC: &[u8] = b"8db42d694ccc418790edff439fe026bf";

    let base = gyroflow_core::filesystem::get_engine_base();
    let mut input = gyroflow_core::filesystem::open_file(&base, input_url, false, false)?;

    let mut buf = vec![0u8; HEADER_SIZE];
    input.seek(SeekFrom::End(-(HEADER_SIZE as i64)))?;
    input.read_exact(&mut buf)?;
    if &buf[HEADER_SIZE-32..] == MAGIC {
        let extra_size = u32::from_le_bytes(buf[32..36].try_into().unwrap()) as i64;
        input.seek(SeekFrom::End(-extra_size))?;

        let mut output = gyroflow_core::filesystem::open_file(&base, output_url, true, false)?;
        output.seek(SeekFrom::End(0))?;
        std::io::copy(&mut input, &mut output.get_file())?;
    }

    Ok(())
}

pub fn report_lens_profile_usage(checksum: Option<String>) {
    if let Some(checksum) = checksum {
        if !checksum.is_empty() {
            gyroflow_core::run_threaded(move || {
                let url = format!("https://api.gyroflow.xyz/usage?checksum={checksum}");

                if let Ok(body) = ureq::get(url).call() {
                    ::log::debug!("Lens profile usage stats: {:?}", body.into_body().read_to_string());
                }
            });
        }
    }
}
