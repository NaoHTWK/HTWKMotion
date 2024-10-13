#pragma once

#include <vector>

class SmoothingFilter {
public:
    explicit SmoothingFilter(float windowSize);
    double predict_next_value(const std::vector<double>& values);
    double smooth(double x);
    float window_size;

private:
    std::vector<double> window_data;

};
