#ifndef TUW_LIBCAMERA__LIBCAMERA_CONVERT_HPP_
#define TUW_LIBCAMERA__LIBCAMERA_CONVERT_HPP_

#include <optional>
#include <unordered_map>
#include <variant>

#include <libcamera/libcamera.h>
#include <sensor_msgs/image_encodings.hpp>

namespace tuw_libcamera {

struct ConvertCommon {
    std::string ros_format;
};

struct ConvertDirect : ConvertCommon {};
struct ConvertCompressed : ConvertCommon {};
struct ConvertDecode : ConvertCommon {};

using FormatMapping =
    std::variant<ConvertDirect, ConvertCompressed, ConvertDecode>;

std::optional<FormatMapping> get_format_mapping(libcamera::PixelFormat fmt,
                                                std::string target_format = "");

}; // namespace tuw_libcamera

#endif // TUW_LIBCAMERA__LIBCAMERA_CONVERT_HPP_