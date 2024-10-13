#pragma once

#include <point_2d.h>
#include <stl_ext.h>

#include <cmath>
#include <set>

struct Position {
    float x{};
    float y{};
    float a{};

    Position() = default;
    Position(float _x, float _y, float _a) : x(_x), y(_y), a(_a) {}
    Position(const htwk::point_2d& p, float a) : x(p.x), y(p.y), a(a) {}

    Position& normalize() {
        float fac = 1.f / std::sqrt(x * x + y * y);
        x *= fac;
        y *= fac;
        return *this;
    }

    htwk::point_2d point() const {
        return {x, y};
    }

    inline float norm() {
        return std::sqrt(x * x + y * y);
    }

    inline float norm_sqr() {
        return x * x + y * y;
    }

    Position mirrored() const {
        return {-x, -y, normalizeRotation(a + M_PIf)};
    }

    void mirror() {
        x = -x;
        y = -y;
        a = normalizeRotation(a + M_PIf);
    }

    bool isAnyNan() const {
        return std::isnan(x) || std::isnan(y) || std::isnan(a);
    }

    float weightedNorm(float alphaWeight) const {
        return std::sqrt(x * x + y * y + alphaWeight * a * a);
    }

    void move(const Position& odo) {
        htwk::point_2d m = odo.point().rotated(a);
        x += m.x;
        y += m.y;
        a = normalizeRotation(a + odo.a);
    }

    Position rotated(float angle) {
        return {point().rotated(angle), a + angle};
    }

    Position& operator+=(const Position& rhs) {
        x += rhs.x;
        y += rhs.y;
        a = normalizeRotation(a + rhs.a);
        return *this;
    }

    Position& operator-=(const Position& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        a = normalizeRotation(a - rhs.a);
        return *this;
    }

    Position& operator*=(float rhs) {
        x *= rhs;
        y *= rhs;
        a = normalizeRotation(a * rhs);
        return *this;
    }

    friend Position operator+(Position lhs, const Position& rhs) {
        lhs += rhs;
        return lhs;
    }

    friend Position operator-(Position lhs, const Position& rhs) {
        lhs -= rhs;
        return lhs;
    }

    friend Position operator*(Position lhs, float rhs) {
        lhs *= rhs;
        return lhs;
    }

    friend Position operator*(float lhs, Position rhs) {
        rhs *= lhs;
        return rhs;
    }

    bool operator==(const Position& rhs) const {
        return x == rhs.x && y == rhs.y && a == rhs.a;
    }
};

struct Hypothesis {
    Position p;
    float qual = 0;
    // Hypotheses with a high trust can move the robot across the field, medium and low hypotheses have to be close to
    // the current belief to be used.
    enum class Trust { HIGH, MEDIUM, LOW };
    Trust trust = Trust::LOW;
    float match_dist = 0;  // Distance of the feature the hypothesis is based on. Further away features influence the
                           // position less than closer ones.
    std::set<size_t> line_ids;
    std::set<size_t> point_feature_ids;

    Hypothesis mirrored() const {
        return {.p = p.mirrored(),
                .qual = qual,
                .trust = trust,
                .match_dist = match_dist,
                .line_ids = line_ids,
                .point_feature_ids = point_feature_ids};
    }
    void mirror() {
        p.mirror();
    }
    int numFeatures() {
        return line_ids.size() + point_feature_ids.size();
    }
};

namespace std {
template <>
struct hash<Position> {
    size_t operator()(const Position& k) const {
        return hash<float>()(k.x) ^ (hash<float>()(k.y) << 1) ^ (hash<float>()(k.a) << 2);
    }
};
}  // namespace std
