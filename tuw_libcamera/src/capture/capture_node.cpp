#include <camera_info_manager/camera_info_manager.hpp>
#include <cstdio>
#include <libcamera/libcamera.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sys/mman.h>
#include <unordered_map>

#include "tuw_libcamera/buffer_context.hpp"
#include "tuw_libcamera/camera_controls_handler.hpp"
#include "tuw_libcamera/convert.hpp"
#include "tuw_libcamera/stream_handler.hpp"

#include "tuw_libcamera_capture_node_parameters.hpp"

namespace tuw_libcamera {

std::ostream &operator<<(std::ostream &out, const libcamera::StreamFormats &f) {
    for (const auto &fmt : f.pixelformats()) {
        out << fmt << " (" << f.range(fmt) << "), ";
    }
    return out;
}

// Provide a single instance of camera manager per process, because only a
// single is supported
static std::mutex cam_manager_mutex;
static std::weak_ptr<libcamera::CameraManager> cam_manager_instance;

std::shared_ptr<libcamera::CameraManager> init_camera_manager() {
    std::lock_guard lock{cam_manager_mutex};
    auto manager = cam_manager_instance.lock();
    if (!manager) {
        manager = std::make_shared<libcamera::CameraManager>();
        manager->start();
        cam_manager_instance = manager;
    }
    return manager;
}

class CaptureNode : public rclcpp::Node {
  public:
    CaptureNode(const rclcpp::NodeOptions &options)
        : Node("libcamera_node", options) {

        param_listener =
            std::make_shared<ParamListener>(get_node_parameters_interface());
        auto params = param_listener->get_params();

        cam_manager = init_camera_manager();
        acquire_camera(params.camera);
        configure_camera(params);
        create_stream_handlers(params);
        size_t num_requests = allocate_buffers();
        create_requests(num_requests);

        if (!params.camera_info_name.empty()) {
            cam_info_manager =
                std::make_unique<camera_info_manager::CameraInfoManager>(
                    this, params.camera_info_name, params.camera_info_url);
            if (cam_info_manager->isCalibrated()) {
                RCLCPP_INFO(get_logger(), "Camera calibration data loaded.");
            }
            cam_info_publisher =
                create_publisher<camera_info_manager::CameraInfo>(
                    "camera_info", rclcpp::SensorDataQoS());
        }

        // Connect to signal, start capture and queue requests
        camera->requestCompleted.connect(this, &CaptureNode::request_completed);
        camera->start();
        for (const auto &request : requests) {
            camera->queueRequest(request.get());
        }

        RCLCPP_INFO(get_logger(), "Started.");
    }

    ~CaptureNode() {
        RCLCPP_INFO(get_logger(), "stopping.");
        // unmap buffers before deallocation
        buffer_ctx.clear();
        if (camera) {
            camera->stop();
            camera->release();
        }
    }

  private:
    std::shared_ptr<ParamListener> param_listener;
    std::shared_ptr<libcamera::CameraManager> cam_manager;
    std::shared_ptr<libcamera::Camera> camera;
    std::unique_ptr<libcamera::CameraConfiguration> config;
    std::unique_ptr<CameraControlsHandler> controls_handler;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator;
    std::vector<std::unique_ptr<libcamera::Request>> requests;
    std::vector<BufferContext> buffer_ctx;
    std::vector<std::unique_ptr<StreamHandler>> stream_handlers;
    std::vector<uint32_t> request_control_seq;

    std::unique_ptr<camera_info_manager::CameraInfoManager> cam_info_manager;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
        cam_info_publisher;

    void add_buffer(size_t stream_idx, libcamera::FrameBuffer *buffer) {
        buffer->setCookie(buffer_ctx.size());
        buffer_ctx.emplace_back(buffer, stream_idx);
    }

    BufferContext &buffer_context(libcamera::FrameBuffer *buffer) {
        return buffer_ctx.at(buffer->cookie());
    }

    void acquire_camera(const std::string &camera_id) {
        if (cam_manager->cameras().empty()) {
            throw std::runtime_error("No cameras found.");
        }

        RCLCPP_INFO(get_logger(), "Available cameras:");
        for (auto const &camera : cam_manager->cameras()) {
            RCLCPP_INFO(get_logger(), "- %s", camera->id().c_str());
        }

        if (camera_id.empty()) {
            camera = cam_manager->cameras().at(0);
        } else {
            camera = cam_manager->get(camera_id);
            if (!camera)
                throw std::runtime_error("Camera with id " + camera_id +
                                         " not found.");
        }

        RCLCPP_INFO(get_logger(), "Selected camera: %s", camera->id().c_str());

        camera->acquire();
    }

    void configure_camera(const Params &params) {
        std::map<std::string, libcamera::StreamRole> role_map{
            {"raw", libcamera::StreamRole::Raw},
            {"video", libcamera::StreamRole::VideoRecording},
            {"still", libcamera::StreamRole::StillCapture},
            {"viewfinder", libcamera::StreamRole::Viewfinder}};
        std::vector<libcamera::StreamRole> roles;
        for (const auto &role_str : params.stream_roles) {
            roles.push_back(role_map.at(role_str));
        }

        config = camera->generateConfiguration(roles);

        if (!config) {
            throw std::runtime_error(
                "No config for configured stream roles found.");
        }

        for (size_t i = 0; i < roles.size(); i++) {
            auto &stream_cfg = config->at(i);
            const auto stream_role = params.stream_roles.at(i);
            const auto &stream_params =
                params.streams.stream_roles_map.at(stream_role);

            if (stream_params.width)
                stream_cfg.size.width = stream_params.width;
            if (stream_params.height)
                stream_cfg.size.height = stream_params.width;
            if (!stream_params.format.empty())
                stream_cfg.pixelFormat =
                    libcamera::PixelFormat::fromString(stream_params.format);
        }

        auto status = config->validate();
        if (status != libcamera::CameraConfiguration::Valid) {
            for (size_t i = 0; i < roles.size(); i++) {
                auto &stream_cfg = config->at(i);
                const auto stream_role = params.stream_roles.at(i);
                RCLCPP_WARN_STREAM(get_logger(), "Available formats for role '"
                                                     << stream_role << "': "
                                                     << stream_cfg.formats());
            }
        }

        if (status == libcamera::CameraConfiguration::Invalid) {
            throw std::runtime_error("Failed to validate configuration.");
        }

        if (status == libcamera::CameraConfiguration::Adjusted) {
            RCLCPP_WARN(get_logger(), "Stream configuration adjusted.");
        }

        for (const auto &stream_config : *config) {
            RCLCPP_INFO(get_logger(), "Configured stream: %s",
                        stream_config.toString().c_str());
        }

        if (camera->configure(config.get()) < 0) {
            throw std::runtime_error("Failed to configure camera.");
        }

        controls_handler =
            std::make_unique<CameraControlsHandler>(this, camera->controls());
    }

    void create_stream_handlers(const Params &params) {
        for (size_t i = 0; i < config->size(); i++) {
            auto &stream_cfg = config->at(i);
            const auto stream_role = params.stream_roles.at(i);
            const auto &stream_params =
                params.streams.stream_roles_map.at(stream_role);

            stream_handlers.push_back(
                create_stream_handler(this, stream_cfg, stream_params));
        }
    }

    size_t allocate_buffers() {
        allocator = std::make_unique<libcamera::FrameBufferAllocator>(camera);
        size_t num_requests = SIZE_MAX;
        for (const auto &stream_config : *config) {
            // Allocate buffers for each stream
            if (allocator->allocate(stream_config.stream()) < 0) {
                throw std::runtime_error(
                    "Failed to allocate buffers for config: " +
                    stream_config.toString());
            }
            size_t allocated =
                allocator->buffers(stream_config.stream()).size();
            RCLCPP_INFO(get_logger(), "Allocated %zu buffers for stream.",
                        allocated);

            // Get minimum number of buffers allocated across each stream
            if (allocated < num_requests) {
                num_requests = allocated;
            }
        }

        if (num_requests == 0 || num_requests == SIZE_MAX)
            throw std::runtime_error("No buffers allocated.");
        return num_requests;
    }

    void create_requests(size_t num_requests) {
        for (size_t req_idx = 0; req_idx < num_requests; req_idx++) {
            // Create request and link buffers
            auto request = camera->createRequest(request_control_seq.size());
            request_control_seq.push_back(0);
            if (!request) {
                throw std::runtime_error("Failed to create request");
            }

            for (size_t stream_idx = 0; stream_idx < config->size();
                 stream_idx++) {
                const auto &stream_cfg = config->at(stream_idx);
                const auto &buffer =
                    allocator->buffers(stream_cfg.stream()).at(req_idx);

                // Add buffer to context
                add_buffer(stream_idx, buffer.get());

                if (request->addBuffer(stream_cfg.stream(), buffer.get()) < 0) {
                    throw std::runtime_error("Failed to add buffer to request");
                }
            }

            // Initialize request controls
            controls_handler->update_request(
                request->controls(), &request_control_seq.at(request->cookie()),
                true);

            // Store request pointer reference
            requests.push_back(std::move(request));
        }
    }

    void request_completed(libcamera::Request *request) {
        if (request->status() == libcamera::Request::RequestCancelled)
            return;

        std_msgs::msg::Header header;
        header.stamp = now();
        for (auto [stream, buffer] : request->buffers()) {
            auto &ctx = buffer_context(buffer);
            stream_handlers.at(ctx.stream_idx())->publish_buffer(ctx, header);
        }

        if (cam_info_manager) {
            auto info = cam_info_manager->getCameraInfo();
            info.header = header;
            cam_info_publisher->publish(info);
        }

        request->reuse(libcamera::Request::ReuseBuffers);
        controls_handler->update_request(
            request->controls(), &request_control_seq.at(request->cookie()));
        camera->queueRequest(request);
    }
};

}; // namespace tuw_libcamera

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tuw_libcamera::CaptureNode)