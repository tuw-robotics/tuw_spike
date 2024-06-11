#ifndef TUW_SPIKE_CAMERA_FILTER_KERNEL_HPP_
#define TUW_SPIKE_CAMERA_FILTER_KERNEL_HPP_

#include <array>
#include <utility>

namespace tuw_camera_laserscan {

template <typename T, size_t N1, size_t N2>
static constexpr std::array<T, N1 + N2 - 1>
convolve(const std::array<T, N1> &a, const std::array<T, N2> &b) {
    static_assert(N1 > 0);
    static_assert(N2 > 0);
    static_assert((N2 % 2) == 1);

    std::array<T, N1 + N2 - 1> result{};
    for (size_t i = 0; i < result.size(); i++) {
        for (size_t j = 0; j < N2; j++) {
            size_t k = i - j;
            result[i] += ((k < N1) ? a[k] : 0) * b[j];
        }
    }
    return result;
}

template <int Iterations = 100>
static constexpr double constexpr_exp(const double x) {
    if (std::is_constant_evaluated()) {
        double accum = 1;
        double result = 0;
        for (int i = 1; i <= Iterations; i++) {
            result += accum;
            accum *= x / i;
        }
        return result;
    } else {
        return exp(x);
    }
}

template <typename T, int N>
static constexpr std::array<T, N> gaussian(T area, const double sigma) {
    static_assert(N > 0);
    static_assert(N % 2 == 1);

    std::array<T, N> result{};
    std::array<double, N> gaussian{};
    double sum = 0.0;
    for (int i = 0; i < N; i++) {
        const int x = i - N / 2;
        gaussian[i] = constexpr_exp(-0.5 * x * x / (sigma * sigma));
        sum += gaussian[i];
    }
    for (int i = 0; i < N; i++) {
        result[i] = static_cast<T>(area * gaussian[i] / sum);
    }
    return result;
}

template <typename TInput, typename TFilter> class FilterLineIterator {
  public:
    FilterLineIterator(const cv::Mat &img, cv::Point start, cv::Point end,
                       std::vector<TFilter> filter_kernel)
        : line(img, std::move(start), std::move(end)),
          kernel(std::move(filter_kernel)),
          // Fill history with values of ray start
          history(kernel.size(), {line.pos(), **line}) {
        if ((kernel.size() % 2) == 0) {
            throw std::runtime_error("Kernel size is not odd");
        }
    }

    template <size_t FilterSize>
    FilterLineIterator(const cv::Mat &img, cv::Point start, cv::Point end,
                       const std::array<TFilter, FilterSize> &filter_kernel)
        : FilterLineIterator(img, std::move(start), std::move(end),
                             {filter_kernel.begin(), filter_kernel.end()}) {}

    std::pair<cv::Point, int16_t> operator*() {
        return {history_val(history.size() / 2 + 1).first, filter_value};
    }

    FilterLineIterator &operator++() {
        TInput value = *reinterpret_cast<const TInput *>(*line);

        // Save historic values for kernel application
        history_val(0) = {line.pos(), value};
        ++index;
        ++line;

        // After this point:
        // history_val(1) is the newest sample
        // history_val(N) is the oldest sample
        // history_val(N/2 + 1) is the sample for which the filter is calculated

        // Only convolve after enough valid values have been read
        if (index >= history.size()) {
            // Apply kernel convolution
            filter_value = 0;
            for (size_t j = 0; j < history.size(); j++) {
                filter_value = cv::saturate_cast<TFilter>(
                    filter_value + history_val(j + 1).second * kernel[j]);
            }
        }

        return *this;
    }

    explicit operator bool() const {
        return index <= static_cast<size_t>(line.count);
    }

  private:
    cv::LineIterator line;
    std::vector<TFilter> kernel;
    std::vector<std::pair<cv::Point, TInput>> history{};
    std::size_t index{};
    TFilter filter_value{};

    [[nodiscard]] size_t history_idx(size_t offset) const {
        return (index - offset) % history.size();
    }

    [[nodiscard]] std::pair<cv::Point, TInput> &history_val(size_t offset) {
        return history.at(history_idx(offset));
    }
};

} // namespace tuw_camera_laserscan

#endif // TUW_SPIKE_CAMERA_FILTER_KERNEL_HPP_
