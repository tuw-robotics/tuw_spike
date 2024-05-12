#ifndef TUW_LIBCAMERA__REQUEST_CONTEXT_HPP_
#define TUW_LIBCAMERA__REQUEST_CONTEXT_HPP_

#include <rclcpp/guard_condition.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/waitable.hpp>

#include <camera_info_manager/camera_info_manager.hpp>

#include <libcamera/request.h>

#include "tuw_libcamera/camera_controls_handler.hpp"
#include "tuw_libcamera/stream_handler.hpp"

namespace tuw_libcamera {

class RequestHandler : public rclcpp::Waitable {
  public:
    using FramePublishedCallback = std::function<void(rclcpp::Time)>;
    RequestHandler(rclcpp::Node *node,
                   std::shared_ptr<libcamera::Camera> camera,
                   size_t num_requests, std::string frame_id);

    void add_to_wait_set(rcl_wait_set_t *wait_set) override;
    bool is_ready(rcl_wait_set_t *wait_set) override;
    std::shared_ptr<void> take_data() override;
    void execute(std::shared_ptr<void> &data) override;
    size_t get_number_of_ready_guard_conditions() override;

    void add_stream(libcamera::Stream *stream,
                    std::unique_ptr<StreamHandler> stream_handler,
                    libcamera::FrameBufferAllocator &allocator);
    void set_frame_publish_callback(FramePublishedCallback callback);
    void start();
    void handle(libcamera::Request *request);

  private:
    struct RequestContext {
        RequestContext(size_t idx, std::shared_ptr<libcamera::Camera> camera);

        std::unique_ptr<libcamera::Request> request;
        uint32_t control_seq{};
        bool waiting{};
        rclcpp::Time stamp{};
    };

    rclcpp::GuardCondition::SharedPtr gc;
    rclcpp::Logger logger;
    rclcpp::Clock::SharedPtr clock;
    std::shared_ptr<libcamera::Camera> camera;
    std::string frame_id;

    std::vector<std::unique_ptr<libcamera::Request>> requests;
    std::vector<BufferContext> buffer_ctx;
    std::vector<RequestContext> request_ctx;
    std::vector<std::unique_ptr<StreamHandler>> stream_handlers;
    std::unique_ptr<CameraControlsHandler> controls_handler;
    FramePublishedCallback callback;

    std::atomic<size_t> waiting_requests = 0;
};

} // namespace tuw_libcamera

#endif // TUW_LIBCAMERA__REQUEST_CONTEXT_HPP_
