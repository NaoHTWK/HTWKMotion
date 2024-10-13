#pragma once

#include <array>

struct RGB {
    float r{0}, g{0}, b{0};
    static const RGB RED;
    static const RGB ORANGE;
    static const RGB GREEN;
    static const RGB BLUE;
    static const RGB YELLOW;
    static const RGB BLACK;
    static const RGB PURPLE;

    bool operator==(const RGB& o) const {
        return r == o.r && g == o.g && b == o.b;
    }

    bool operator!=(const RGB& o) const {
        return !(*this == o);
    }
};

struct Leds {
    using Eye = std::array<RGB, 8>;

    /**
     * Lola wants float with 0.f or 1.f please don't set anything else.
     */
    using Skull = std::array<float, 12>;

    struct Eyes {
        Eye left;
        Eye right;
    };

    struct Ears {
        std::array<float, 10> left;
        std::array<float, 10> right;
    };

    struct Feet {
        RGB left;
        RGB right;
    };

    Ears ears {};
    Eyes eyes {};
    Feet feet {};
    RGB chest {};
    Skull skull{};

};
