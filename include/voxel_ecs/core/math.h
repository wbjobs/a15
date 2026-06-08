#pragma once

#include <cstdint>
#include <cmath>
#include <array>
#include <algorithm>
#include <bit>
#include <functional>

namespace voxel_ecs::math {

struct alignas(8) Vec3i64 {
    int64_t x, y, z;
    
    constexpr Vec3i64() : x(0), y(0), z(0) {}
    constexpr Vec3i64(int64_t x, int64_t y, int64_t z) : x(x), y(y), z(z) {}
    
    constexpr Vec3i64 operator+(const Vec3i64& o) const { return {x+o.x, y+o.y, z+o.z}; }
    constexpr Vec3i64 operator-(const Vec3i64& o) const { return {x-o.x, y-o.y, z-o.z}; }
    constexpr Vec3i64 operator*(int64_t s) const { return {x*s, y*s, z*s}; }
    constexpr Vec3i64 operator/(int64_t s) const { return {x/s, y/s, z/s}; }
    
    constexpr bool operator==(const Vec3i64& o) const { return x==o.x && y==o.y && z==o.z; }
    constexpr bool operator!=(const Vec3i64& o) const { return !(*this == o); }
};

struct Vec3f {
    float x, y, z;
    
    constexpr Vec3f() : x(0), y(0), z(0) {}
    constexpr Vec3f(float x, float y, float z) : x(x), y(y), z(z) {}
    
    constexpr Vec3f operator+(const Vec3f& o) const { return {x+o.x, y+o.y, z+o.z}; }
    constexpr Vec3f operator-(const Vec3f& o) const { return {x-o.x, y-o.y, z-o.z}; }
    constexpr Vec3f operator-() const { return {-x, -y, -z}; }
    constexpr Vec3f operator*(float s) const { return {x*s, y*s, z*s}; }
    constexpr Vec3f operator/(float s) const { return {x/s, y/s, z/s}; }
    
    Vec3f& operator+=(const Vec3f& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vec3f& operator-=(const Vec3f& o) { x-=o.x; y-=o.y; z-=o.z; return *this; }
    Vec3f& operator*=(float s) { x*=s; y*=s; z*=s; return *this; }
    Vec3f& operator/=(float s) { x/=s; y/=s; z/=s; return *this; }
    
    float& operator[](int i) { return (&x)[i]; }
    const float& operator[](int i) const { return (&x)[i]; }
    
    float length_sq() const { return x*x + y*y + z*z; }
    float length() const { return std::sqrt(length_sq()); }
    
    Vec3f normalized() const {
        float len = length();
        return len > 1e-8f ? Vec3f(x/len, y/len, z/len) : Vec3f(0, 0, 0);
    }
    
    constexpr bool operator==(const Vec3f& o) const { 
        return std::abs(x-o.x) < 1e-6f && std::abs(y-o.y) < 1e-6f && std::abs(z-o.z) < 1e-6f; 
    }
};

struct alignas(16) Vec4f {
    float x, y, z, w;
    
    constexpr Vec4f() : x(0), y(0), z(0), w(1) {}
    constexpr Vec4f(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
    constexpr Vec4f(const Vec3f& v, float w) : x(v.x), y(v.y), z(v.z), w(w) {}
    
    operator Vec3f() const { return {x, y, z}; }
};

struct Vec2f {
    float x, y;
    
    constexpr Vec2f() : x(0), y(0) {}
    constexpr Vec2f(float x, float y) : x(x), y(y) {}
    
    constexpr Vec2f operator+(const Vec2f& o) const { return {x+o.x, y+o.y}; }
    constexpr Vec2f operator-(const Vec2f& o) const { return {x-o.x, y-o.y}; }
    constexpr Vec2f operator*(float s) const { return {x*s, y*s}; }
    constexpr Vec2f operator/(float s) const { return {x/s, y/s}; }
    
    Vec2f& operator+=(const Vec2f& o) { x+=o.x; y+=o.y; return *this; }
    Vec2f& operator*=(float s) { x*=s; y*=s; return *this; }
    
    float length_sq() const { return x*x + y*y; }
    float length() const { return std::sqrt(length_sq()); }
    
    Vec2f normalized() const {
        float len = length();
        return len > 1e-8f ? Vec2f(x/len, y/len) : Vec2f(0, 0);
    }
    
    constexpr bool operator==(const Vec2f& o) const {
        return std::abs(x-o.x) < 1e-5f && std::abs(y-o.y) < 1e-5f;
    }
};

struct Color {
    float r, g, b;
    
    constexpr Color() : r(0), g(0), b(0) {}
    constexpr Color(float r, float g, float b) : r(r), g(g), b(b) {}
    
    static constexpr Color from_rgb(uint8_t r, uint8_t g, uint8_t b) {
        return {r / 255.0f, g / 255.0f, b / 255.0f};
    }
    
    uint8_t r8() const { return static_cast<uint8_t>(std::clamp(r, 0.0f, 1.0f) * 255); }
    uint8_t g8() const { return static_cast<uint8_t>(std::clamp(g, 0.0f, 1.0f) * 255); }
    uint8_t b8() const { return static_cast<uint8_t>(std::clamp(b, 0.0f, 1.0f) * 255); }
    
    Color operator+(const Color& o) const { return {r+o.r, g+o.g, b+o.b}; }
    Color operator-(const Color& o) const { return {r-o.r, g-o.g, b-o.b}; }
    Color operator*(float s) const { return {r*s, g*s, b*s}; }
    Color operator/(float s) const { return {r/s, g/s, b/s}; }
    Color operator*(const Color& o) const { return {r*o.r, g*o.g, b*o.b}; }
    Color& operator+=(const Color& o) { r+=o.r; g+=o.g; b+=o.b; return *this; }
    Color& operator*=(float s) { r*=s; g*=s; b*=s; return *this; }
    
    bool operator==(const Color& o) const {
        return std::abs(r - o.r) < 1e-5f && 
               std::abs(g - o.g) < 1e-5f && 
               std::abs(b - o.b) < 1e-5f;
    }
    
    bool operator!=(const Color& o) const {
        return !(*this == o);
    }
};

struct AABB {
    Vec3f min, max;
    
    AABB() : min(1e30f, 1e30f, 1e30f), max(-1e30f, -1e30f, -1e30f) {}
    AABB(const Vec3f& min, const Vec3f& max) : min(min), max(max) {}
    
    void expand(const Vec3f& p) {
        min.x = std::min(min.x, p.x);
        min.y = std::min(min.y, p.y);
        min.z = std::min(min.z, p.z);
        max.x = std::max(max.x, p.x);
        max.y = std::max(max.y, p.y);
        max.z = std::max(max.z, p.z);
    }
    
    bool contains(const Vec3f& p) const {
        return p.x >= min.x && p.x <= max.x && 
               p.y >= min.y && p.y <= max.y && 
               p.z >= min.z && p.z <= max.z;
    }
    
    bool intersects(const AABB& o) const {
        return min.x <= o.max.x && max.x >= o.min.x &&
               min.y <= o.max.y && max.y >= o.min.y &&
               min.z <= o.max.z && max.z >= o.min.z;
    }
};

inline float dot(const Vec3f& a, const Vec3f& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

inline float dot(const Vec2f& a, const Vec2f& b) {
    return a.x*b.x + a.y*b.y;
}

inline Vec3f cross(const Vec3f& a, const Vec3f& b) {
    return {
        a.y*b.z - a.z*b.y,
        a.z*b.x - a.x*b.z,
        a.x*b.y - a.y*b.x
    };
}

inline Vec3f reflect(const Vec3f& v, const Vec3f& n) {
    return v - n * (2.0f * dot(v, n));
}

inline Vec3f operator*(float s, const Vec3f& v) {
    return {s * v.x, s * v.y, s * v.z};
}

inline float length_sq(const Vec3f& v) {
    return dot(v, v);
}

inline float length(const Vec3f& v) {
    return std::sqrt(length_sq(v));
}

inline Vec3f normalize(const Vec3f& v) {
    float len = length(v);
    return len > 1e-8f ? v / len : Vec3f(0, 0, 0);
}

inline Vec3f lerp(const Vec3f& a, const Vec3f& b, float t) {
    return a + (b - a) * t;
}

inline Color lerp(const Color& a, const Color& b, float t) {
    return a + (b - a) * t;
}

inline uint64_t hash_coords(int64_t x, int64_t y, int64_t z) {
    uint64_t h = 0x9e3779b97f4a7c15ULL;
    h ^= static_cast<uint64_t>(x) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    h ^= static_cast<uint64_t>(y) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    h ^= static_cast<uint64_t>(z) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    return h;
}

inline uint64_t hash_vec(const Vec3i64& v) {
    return hash_coords(v.x, v.y, v.z);
}

constexpr int32_t floor_div(int32_t x, int32_t y) {
    return (x >= 0) ? (x / y) : ((x - y + 1) / y);
}

constexpr int64_t floor_div(int64_t x, int64_t y) {
    return (x >= 0) ? (x / y) : ((x - y + 1) / y);
}

constexpr Vec3i64 floor_div(const Vec3i64& v, int64_t s) {
    return {floor_div(v.x, s), floor_div(v.y, s), floor_div(v.z, s)};
}

}

namespace std {
template <>
struct hash<voxel_ecs::math::Vec3i64> {
    size_t operator()(const voxel_ecs::math::Vec3i64& v) const noexcept {
        return static_cast<size_t>(voxel_ecs::math::hash_vec(v));
    }
};
}
