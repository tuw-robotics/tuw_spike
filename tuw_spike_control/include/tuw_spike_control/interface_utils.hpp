#ifndef TUW_SPIKE_CONTROL__INTERFACE_UTILS_HPP_
#define TUW_SPIKE_CONTROL__INTERFACE_UTILS_HPP_

#include <type_traits>
#include <charconv>

namespace tuw_spike_control {

template <typename T,
          std::enable_if_t<std::is_assignable_v<T, std::string>, bool> = true>
T read_param_impl(const std::unordered_map<std::string, std::string> &params,
                  const std::string &param) {
    auto result = params.find(param);
    if (result == params.end()) {
        throw std::runtime_error("Parameter '" + param + "' not set");
    }
    return result->second;
}

template <typename T, std::enable_if_t<std::is_integral_v<T>, bool> = true,
          std::enable_if_t<!std::is_same_v<T, bool>, bool> = true>
T read_param_impl(const std::unordered_map<std::string, std::string> &params,
                  const std::string &param) {
    T value = 0;
    int base = 10;

    auto string = read_param_impl<std::string>(params, param);
    auto start = string.c_str();

    if (string.size() >= 2 && string.substr(0, 2) == "0x") {
        base = 16;
        start += 2;
    }

    auto [ptr, ec] =
        std::from_chars(start, string.c_str() + string.size(), value, base);
    if (ec != std::errc() || ptr != string.c_str() + string.size()) {
        throw std::runtime_error("Parameter '" + param +
                                 "' is not a integer in the valid range");
    }

    return value;
}

template <typename T, std::enable_if_t<std::is_same_v<T, double>, bool> = true>
T read_param_impl(const std::unordered_map<std::string, std::string> &params,
                  const std::string &param) {
    T value = 0.0;
    auto string = read_param_impl<std::string>(params, param);

    auto [ptr, ec] =
        std::from_chars(string.c_str(), string.c_str() + string.size(), value);
    if (ec != std::errc() || ptr != string.c_str() + string.size()) {
        throw std::runtime_error("Parameter '" + param +
                                 "' is not a double in the valid range");
    }

    return value;
}

template <typename T, std::enable_if_t<std::is_same_v<T, bool>, bool> = true>
T read_param_impl(const std::unordered_map<std::string, std::string> &params,
                  const std::string &param) {
    auto string = read_param_impl<std::string>(params, param);

    if (string == "true") {
        return true;
    } else if (string == "false") {
        return false;
    } else {
        throw std::runtime_error(
            "Parameter '" + param +
            "' is not a bool in the valid range (true / false)");
    }
}

template <typename T>
T read_param(const std::unordered_map<std::string, std::string> &params,
             const std::string &param, std::optional<T> def = std::nullopt) {
    if (def && !params.contains(param))
        return *def;
    return read_param_impl<T>(params, param);
}

template <typename T>
T read_param(const hardware_interface::HardwareInfo &info,
             const std::string &param, std::optional<T> def = std::nullopt) {
    return read_param<T>(info.hardware_parameters, param, def);
}

template <typename T>
T read_param(const hardware_interface::ComponentInfo &info,
             const std::string &param, std::optional<T> def = std::nullopt) {
    return read_param<T>(info.parameters, param, def);
}

} // namespace tuw_spike_control

#endif // TUW_SPIKE_CONTROL__INTERFACE_UTILS_HPP_
