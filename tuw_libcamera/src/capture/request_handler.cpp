#include "tuw_libcamera/request_handler.hpp"
#include "rcutils/logging_macros.h"
#include <thread>

namespace tuw_libcamera {

RequestHandler::RequestContext::RequestContext(
    size_t idx, std::shared_ptr<libcamera::Camera> camera) {
    request = camera->createRequest(idx);
    if (!request) {
        throw std::runtime_error("Failed to create request");
    }
}

RequestHandler::RequestHandler(rclcpp::Node *node,
                               std::shared_ptr<libcamera::Camera> camera,
                               size_t num_requests, std::string frame_id)
    : logger(node->get_logger()), clock(node->get_clock()), camera(camera),
      frame_id(frame_id) {
    static_assert(std::numeric_limits<size_t>::max() <=
                  std::numeric_limits<uint64_t>::max());

    gc = std::make_shared<rclcpp::GuardCondition>(
        node->get_node_base_interface()->get_context());
    controls_handler =
        std::make_unique<CameraControlsHandler>(node, camera->controls());

    for (size_t request_idx = 0; request_idx < num_requests; request_idx++) {
        auto &ctx = request_ctx.emplace_back(request_idx, camera);
        // Initialize request controls
        controls_handler->update_request(ctx.request->controls(),
                                         &ctx.control_seq, true);
    }
}

void RequestHandler::add_stream(libcamera::Stream *stream,
                                std::unique_ptr<StreamHandler> stream_handler,
                                libcamera::FrameBufferAllocator &allocator) {
    size_t stream_idx = stream_handlers.size();
    for (size_t request_idx = 0; request_idx < request_ctx.size();
         request_idx++) {
        const auto &buffer = allocator.buffers(stream).at(request_idx);

        // Add buffer to context
        size_t buffer_idx = buffer_ctx.size();
        buffer->setCookie(buffer_idx);
        buffer_ctx.emplace_back(buffer.get(), stream_idx);

        // Add buffer to request
        if (request_ctx.at(request_idx)
                .request->addBuffer(stream, buffer.get()) < 0) {
            throw std::runtime_error("Failed to add buffer to request");
        }
    }
    stream_handlers.push_back(std::move(stream_handler));
}

void RequestHandler::set_frame_publish_callback(
    FramePublishedCallback callback) {
    this->callback = callback;
}

void RequestHandler::start() {
    for (const auto &ctx : request_ctx) {
        camera->queueRequest(ctx.request.get());
    }
}

void RequestHandler::handle(libcamera::Request *request) {
    if (request->status() == libcamera::Request::RequestCancelled)
        return;

    RCLCPP_DEBUG_STREAM(logger, "Received completed request "
                                    << request->cookie() << " on thread: "
                                    << std::this_thread::get_id());

    auto &ctx = request_ctx.at(request->cookie());
    if (ctx.waiting)
        throw std::runtime_error("assertion failed: request not processed");
    ctx.stamp = clock->now();
    ctx.waiting = true;
    waiting_requests++;
    gc->trigger();
}

void RequestHandler::execute(std::shared_ptr<void> &data) {
    (void)data;
    RCLCPP_DEBUG(logger, "Execute begin: waiting = %ld",
                 waiting_requests.load());
    for (auto &ctx : request_ctx) {
        if (ctx.waiting) {
            waiting_requests--;

            RCLCPP_DEBUG_STREAM(logger, "Handle completed request "
                                            << ctx.request->cookie()
                                            << " on thread: "
                                            << std::this_thread::get_id()
                                            << " with stamp: "
                                            << (long)(ctx.stamp.seconds()*1e6));

            std_msgs::msg::Header header;
            header.stamp = ctx.stamp;
            header.frame_id = frame_id;
            for (auto [stream, buffer] : ctx.request->buffers()) {
                auto &ctx = buffer_ctx.at(buffer->cookie());
                stream_handlers.at(ctx.stream_idx())
                    ->publish_buffer(ctx, header);
            }

            RCLCPP_DEBUG_STREAM(logger, "Published completed request "
                                            << ctx.request->cookie());

            if (callback) {
                callback(ctx.stamp);
            }

            ctx.request->reuse(libcamera::Request::ReuseBuffers);
            controls_handler->update_request(ctx.request->controls(),
                                             &ctx.control_seq, false);
            ctx.waiting = false;
            camera->queueRequest(ctx.request.get());
        }
    }
    RCLCPP_DEBUG(logger, "Execute end: waiting = %ld", waiting_requests.load());
}

void RequestHandler::add_to_wait_set(rcl_wait_set_t *wait_set) {
    gc->add_to_wait_set(wait_set);
}

bool RequestHandler::is_ready(rcl_wait_set_t *wait_set) {
    (void)wait_set;
    RCLCPP_DEBUG(logger, "is_ready: waiting = %ld", waiting_requests.load());
    return waiting_requests > 0;
}

std::shared_ptr<void> RequestHandler::take_data() { return nullptr; }

size_t RequestHandler::get_number_of_ready_guard_conditions() { return 1; }

} // namespace tuw_libcamera