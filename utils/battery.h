#pragma once

struct Battery {
    float current{};  // in A, negative for discharge, positive for charge
    float temp{};  // 10 deg Celsius
    float status{};  // some enum/bit field
    float charge{};  // 0.99 or 1 for fully charged
};
