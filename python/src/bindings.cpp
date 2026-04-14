// Python bindings for the INDEMIND IMSEE-SDK.
//
// Scope: raw stereo image + IMU + module calibration. No SLAM, no depth/
// disparity, no detector. The binding intentionally avoids cv::Mat on the
// API edge -- it uses RegistModuleCameraCallback (raw uint8 buffers) so no
// OpenCV types cross the extension's ABI boundary.
//
// The SDK's image/IMU callbacks fire on a C++ worker thread. We copy the
// bytes into a std::vector without holding the GIL, enqueue under a mutex,
// and convert to numpy only inside get_frame()/drain_imu() when the GIL is
// already held.

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <cmath>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "imrsdk.h"
#include "types.h"

namespace py = pybind11;

namespace {

constexpr float kG = 9.81f;
constexpr float kDeg2Rad = static_cast<float>(M_PI / 180.0);

struct RawFrame {
    double timestamp{0.0};
    int width{0};
    int height{0};
    int channels{0};
    std::vector<uint8_t> left;
    std::vector<uint8_t> right;
};

struct ImuSample {
    double timestamp{0.0};
    float accel[3]{0, 0, 0};
    float gyro[3]{0, 0, 0};
};

class Camera {
public:
    Camera() = default;

    ~Camera() { stop(); }

    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;

    bool start(const std::string& resolution,
               int img_hz,
               int imu_hz,
               bool si_units,
               std::size_t frame_queue,
               std::size_t imu_queue) {
        if (running_) return true;

        si_units_ = si_units;
        max_frames_ = frame_queue;
        max_imu_ = imu_queue;

        indem::MRCONFIG cfg = {0};
        cfg.bSlam = false;
        cfg.imgFrequency = img_hz;
        cfg.imuFrequency = imu_hz;
        if (resolution == "640x400" || resolution == "640") {
            cfg.imgResolution = indem::IMG_640;
        } else if (resolution == "1280x800" || resolution == "1280") {
            cfg.imgResolution = indem::IMG_1280;
        } else {
            throw std::invalid_argument(
                "resolution must be '640x400' or '1280x800' (got '" +
                resolution + "')");
        }

        sdk_ = new indem::CIMRSDK();

        sdk_->RegistModuleCameraCallback(
            [this](double t, unsigned char* pL, unsigned char* pR,
                   int w, int h, int c, void* /*user*/) {
                this->on_image(t, pL, pR, w, h, c);
            },
            nullptr);

        sdk_->RegistModuleIMUCallback(
            [this](indem::ImuData imu) { this->on_imu(imu); });

        bool ok;
        {
            py::gil_scoped_release release;
            ok = sdk_->Init(cfg);
        }
        if (!ok) {
            delete sdk_;
            sdk_ = nullptr;
            return false;
        }
        running_ = true;
        return true;
    }

    void stop() {
        if (!running_ && sdk_ == nullptr) return;
        running_ = false;
        {
            py::gil_scoped_release release;
            delete sdk_;
        }
        sdk_ = nullptr;
        std::lock_guard<std::mutex> lk(img_mu_);
        frames_.clear();
        img_cv_.notify_all();
    }

    bool is_running() const { return running_; }

    py::object get_frame(double timeout_s) {
        RawFrame f;
        {
            std::unique_lock<std::mutex> lk(img_mu_);
            const auto dur = std::chrono::duration<double>(timeout_s);
            bool have_frame;
            {
                py::gil_scoped_release release;
                have_frame = img_cv_.wait_for(
                    lk, dur,
                    [this] { return !frames_.empty() || !running_; });
            }
            if (!have_frame || frames_.empty()) return py::none();
            f = std::move(frames_.front());
            frames_.pop_front();
        }

        std::vector<ssize_t> shape = (f.channels == 1)
            ? std::vector<ssize_t>{f.height, f.width}
            : std::vector<ssize_t>{f.height, f.width, f.channels};

        py::array_t<uint8_t> left(shape);
        py::array_t<uint8_t> right(shape);
        std::memcpy(left.mutable_data(), f.left.data(), f.left.size());
        std::memcpy(right.mutable_data(), f.right.data(), f.right.size());
        return py::make_tuple(f.timestamp, left, right);
    }

    std::vector<ImuSample> drain_imu() {
        std::lock_guard<std::mutex> lk(imu_mu_);
        std::vector<ImuSample> out(imu_.begin(), imu_.end());
        imu_.clear();
        return out;
    }

    py::dict get_module_params() {
        if (sdk_ == nullptr) {
            throw std::runtime_error("Camera is not started");
        }
        indem::MoudleAllParam p = sdk_->GetModuleParams();
        const auto& im = p._imu;
        const auto& dev = p._device;

        py::dict d;

        py::dict imu;
        imu["a_max"] = im._a_max;
        imu["g_max"] = im._g_max;
        imu["sigma_g_c"] = im._sigma_g_c;
        imu["sigma_a_c"] = im._sigma_a_c;
        imu["sigma_bg"] = im._sigma_bg;
        imu["sigma_ba"] = im._sigma_ba;
        imu["sigma_gw_c"] = im._sigma_gw_c;
        imu["sigma_aw_c"] = im._sigma_aw_c;
        imu["g"] = im._g;
        imu["T_BS"] = py::array_t<double>({4, 4}, im._T_BS);
        d["imu"] = imu;

        py::dict device;
        device["id"] = std::string(dev._id);
        device["designer"] = std::string(dev._designer);
        device["firmware_version"] = std::string(dev._fireware_version);
        device["hardware_version"] = std::string(dev._hardware_version);
        device["lens"] = std::string(dev._lens);
        device["imu_chip"] = std::string(dev._imu);
        device["viewing_angle"] = std::string(dev._viewing_angle);
        device["baseline_str"] = std::string(dev._baseline);
        d["device"] = device;

        d["baseline_m"] = p._baseline;
        d["camera_channel"] = p._camera_channel;
        return d;
    }

    py::dict get_module_info() {
        if (sdk_ == nullptr) {
            throw std::runtime_error("Camera is not started");
        }
        indem::ModuleInfo info = sdk_->GetModuleInfo();
        py::dict d;
        d["id"] = std::string(info._id);
        d["designer"] = std::string(info._designer);
        d["firmware_version"] = std::string(info._fireware_version);
        d["hardware_version"] = std::string(info._hardware_version);
        d["lens"] = std::string(info._lens);
        d["imu_chip"] = std::string(info._imu);
        d["viewing_angle"] = std::string(info._viewing_angle);
        d["baseline_str"] = std::string(info._baseline);
        return d;
    }

private:
    void on_image(double t, unsigned char* pL, unsigned char* pR,
                  int w, int h, int c) {
        const std::size_t n = static_cast<std::size_t>(w) * h * c;
        RawFrame f;
        f.timestamp = t;
        f.width = w;
        f.height = h;
        f.channels = c;
        f.left.assign(pL, pL + n);
        f.right.assign(pR, pR + n);
        {
            std::lock_guard<std::mutex> lk(img_mu_);
            while (frames_.size() >= max_frames_) frames_.pop_front();
            frames_.push_back(std::move(f));
        }
        img_cv_.notify_one();
    }

    void on_imu(indem::ImuData imu) {
        ImuSample s;
        s.timestamp = imu.timestamp;
        if (si_units_) {
            s.accel[0] = imu.accel[0] * kG;
            s.accel[1] = imu.accel[1] * kG;
            s.accel[2] = imu.accel[2] * kG;
            s.gyro[0] = imu.gyro[0] * kDeg2Rad;
            s.gyro[1] = imu.gyro[1] * kDeg2Rad;
            s.gyro[2] = imu.gyro[2] * kDeg2Rad;
        } else {
            s.accel[0] = imu.accel[0]; s.accel[1] = imu.accel[1]; s.accel[2] = imu.accel[2];
            s.gyro[0]  = imu.gyro[0];  s.gyro[1]  = imu.gyro[1];  s.gyro[2]  = imu.gyro[2];
        }
        std::lock_guard<std::mutex> lk(imu_mu_);
        while (imu_.size() >= max_imu_) imu_.pop_front();
        imu_.push_back(s);
    }

    indem::CIMRSDK* sdk_{nullptr};
    std::atomic<bool> running_{false};
    bool si_units_{true};
    std::size_t max_frames_{4};
    std::size_t max_imu_{4000};

    std::deque<RawFrame> frames_;
    std::deque<ImuSample> imu_;
    std::mutex img_mu_;
    std::mutex imu_mu_;
    std::condition_variable img_cv_;
};

}  // namespace

PYBIND11_MODULE(_pyindemind_ext, m) {
    m.doc() = "pybind11 bindings for the INDEMIND IMSEE-SDK";

    py::class_<ImuSample>(m, "ImuSample")
        .def_readonly("timestamp", &ImuSample::timestamp)
        .def_property_readonly("accel", [](const ImuSample& s) {
            return py::array_t<float>({3}, s.accel);
        })
        .def_property_readonly("gyro", [](const ImuSample& s) {
            return py::array_t<float>({3}, s.gyro);
        })
        .def("__repr__", [](const ImuSample& s) {
            char buf[160];
            std::snprintf(buf, sizeof(buf),
                "ImuSample(t=%.6f, accel=(%.4f, %.4f, %.4f), gyro=(%.4f, %.4f, %.4f))",
                s.timestamp, s.accel[0], s.accel[1], s.accel[2],
                s.gyro[0], s.gyro[1], s.gyro[2]);
            return std::string(buf);
        });

    py::class_<Camera>(m, "Camera")
        .def(py::init<>())
        .def("start", &Camera::start,
             py::arg("resolution") = "640x400",
             py::arg("img_hz") = 50,
             py::arg("imu_hz") = 1000,
             py::arg("si_units") = true,
             py::arg("frame_queue") = 4,
             py::arg("imu_queue") = 4000)
        .def("stop", &Camera::stop)
        .def("is_running", &Camera::is_running)
        .def("get_frame", &Camera::get_frame, py::arg("timeout_s") = 0.1)
        .def("drain_imu", &Camera::drain_imu)
        .def("get_module_params", &Camera::get_module_params)
        .def("get_module_info", &Camera::get_module_info);
}
