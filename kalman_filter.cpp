#include "kalman_filter.h"

SmoothingFilter::SmoothingFilter(float windowSize) : window_size(windowSize) {}

double SmoothingFilter::predict_next_value(const std::vector<double>& values) {
    int n = values.size();
    double sum_x = 0;
    double sum_y = 0;
    double sum_x2 = 0;
    double sum_xy = 0;

    for (int i = 0; i < n; ++i) {
        sum_x += i;
        sum_y += values[i];
        sum_x2 += i * i;
        sum_xy += i * values[i];
    }

    double a = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);
    double b = (sum_y - a * sum_x) / n;

    double next_x = n - 1;
    double next_value = a * next_x + b;

    return next_value;
}

double SmoothingFilter::smooth(double x) {
    window_data.push_back(x);
    if (window_data.size() > window_size+0.5) {
        window_data.erase(window_data.begin());
        return predict_next_value(window_data);
    }
    return x;
}
