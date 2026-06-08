#pragma once

#include "../core/math.h"
#include "../voxel/voxel.h"
#include "../gi/ddgi.h"
#include "../ecs/ecs.h"
#include <vector>
#include <cassert>
#include <array>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace voxel_ecs::render {

using namespace math;
using namespace voxel;
using namespace gi;

class SoftwareRenderer {
public:
    SoftwareRenderer(uint32_t width, uint32_t height)
        : width_(width), height_(height), framebuffer_(width * height * 4) {}
    
    void set_size(uint32_t width, uint32_t height) {
        width_ = width;
        height_ = height;
        framebuffer_.resize(width * height * 4);
    }
    
    void clear(const Color& color = Color(0.1f, 0.1f, 0.15f)) {
        for (uint32_t y = 0; y < height_; ++y) {
            for (uint32_t x = 0; x < width_; ++x) {
                uint32_t idx = (y * width_ + x) * 4;
                framebuffer_[idx]     = color.r8();
                framebuffer_[idx + 1] = color.g8();
                framebuffer_[idx + 2] = color.b8();
                framebuffer_[idx + 3] = 255;
            }
        }
    }
    
    void render(const VoxelWorld& world, const DDGIComponent* ddgi,
                const Vec3f& camera_pos, const Vec3f& camera_dir, 
                float fov = 60.0f) {
        clear();
        
        float aspect = static_cast<float>(width_) / static_cast<float>(height_);
        float tan_fov = std::tan(fov * 0.5f * 3.14159f / 180.0f);
        
        Vec3f forward = normalize(camera_dir);
        Vec3f right = normalize(cross(Vec3f(0, 1, 0), forward));
        Vec3f up = cross(forward, right);
        
        #pragma omp parallel for
        for (int64_t y = 0; y < static_cast<int64_t>(height_); ++y) {
            for (uint32_t x = 0; x < width_; ++x) {
                float u = (2.0f * static_cast<float>(x) / static_cast<float>(width_) - 1.0f) * tan_fov * aspect;
                float v = (1.0f - 2.0f * static_cast<float>(y) / static_cast<float>(height_)) * tan_fov;
                
                Vec3f ray_dir = normalize(forward + right * u + up * v);
                
                Color pixel_color = trace_ray(world, ddgi, camera_pos, ray_dir, 100.0f);
                
                uint32_t idx = (y * width_ + x) * 4;
                framebuffer_[idx]     = pixel_color.r8();
                framebuffer_[idx + 1] = pixel_color.g8();
                framebuffer_[idx + 2] = pixel_color.b8();
                framebuffer_[idx + 3] = 255;
            }
        }
    }
    
    const std::vector<uint8_t>& framebuffer() const { return framebuffer_; }
    uint32_t width() const { return width_; }
    uint32_t height() const { return height_; }
    
private:
    Color trace_ray(const VoxelWorld& world, const DDGIComponent* ddgi,
                    const Vec3f& origin, const Vec3f& dir, float max_dist) {
        Vec3f hit_pos;
        Vec3i64 hit_voxel;
        
        if (!raycast(world, origin, dir, max_dist, hit_pos, hit_voxel)) {
            float t = (dir.y + 1.0f) * 0.5f;
            return lerp(Color(0.5f, 0.7f, 0.9f), Color(0.1f, 0.1f, 0.2f), t);
        }
        
        auto voxel = world.get_voxel(hit_voxel);
        if (!voxel) return Color(0, 0, 0);
        
        Vec3f normal = compute_normal(world, hit_pos, dir);
        Color albedo = voxel->albedo;
        
        Color direct_light = compute_direct_lighting(world, hit_pos + normal * 0.01f, normal);
        
        Color indirect_light(0, 0, 0);
        if (ddgi) {
            GISystem gi;
            indirect_light = gi.compute_indirect_lighting(
                hit_pos + normal * 0.01f, normal, -dir, *ddgi
            );
        } else {
            indirect_light = compute_ambient_occlusion(world, hit_pos + normal * 0.01f, normal);
        }
        
        Color ambient = albedo * 0.1f;
        Color result = albedo * (direct_light + ambient) + indirect_light * albedo;
        
        if (voxel->emission > 0.0f) {
            result += albedo * voxel->emission;
        }
        
        return tone_map(result);
    }
    
    bool raycast(const VoxelWorld& world, const Vec3f& origin, const Vec3f& dir, 
                 float max_dist, Vec3f& hit_pos, Vec3i64& hit_voxel) const {
        Vec3i64 step(
            dir.x > 0 ? 1 : -1,
            dir.y > 0 ? 1 : -1,
            dir.z > 0 ? 1 : -1
        );
        
        Vec3f t_delta(
            std::abs(dir.x) > 1e-6f ? 1.0f / std::abs(dir.x) : 1e30f,
            std::abs(dir.y) > 1e-6f ? 1.0f / std::abs(dir.y) : 1e30f,
            std::abs(dir.z) > 1e-6f ? 1.0f / std::abs(dir.z) : 1e30f
        );
        
        Vec3i64 voxel_pos(
            static_cast<int64_t>(std::floor(origin.x)),
            static_cast<int64_t>(std::floor(origin.y)),
            static_cast<int64_t>(std::floor(origin.z))
        );
        
        Vec3f t_max(
            (step.x > 0 ? (voxel_pos.x + 1 - origin.x) : (origin.x - voxel_pos.x)) * t_delta.x,
            (step.y > 0 ? (voxel_pos.y + 1 - origin.y) : (origin.y - voxel_pos.y)) * t_delta.y,
            (step.z > 0 ? (voxel_pos.z + 1 - origin.z) : (origin.z - voxel_pos.z)) * t_delta.z
        );
        
        float t = 0;
        while (t < max_dist) {
            auto voxel = world.get_voxel(voxel_pos);
            if (voxel && voxel->is_solid()) {
                hit_pos = origin + dir * t;
                hit_voxel = voxel_pos;
                return true;
            }
            
            if (t_max.x < t_max.y) {
                if (t_max.x < t_max.z) {
                    t = t_max.x;
                    voxel_pos.x += step.x;
                    t_max.x += t_delta.x;
                } else {
                    t = t_max.z;
                    voxel_pos.z += step.z;
                    t_max.z += t_delta.z;
                }
            } else {
                if (t_max.y < t_max.z) {
                    t = t_max.y;
                    voxel_pos.y += step.y;
                    t_max.y += t_delta.y;
                } else {
                    t = t_max.z;
                    voxel_pos.z += step.z;
                    t_max.z += t_delta.z;
                }
            }
        }
        
        return false;
    }
    
    Vec3f compute_normal(const VoxelWorld& world, const Vec3f& pos, const Vec3f& incoming) const {
        Vec3f normal(0, 0, 0);
        
        for (int axis = 0; axis < 3; ++axis) {
            for (int sign = -1; sign <= 1; sign += 2) {
                Vec3f offset(0, 0, 0);
                offset[axis] = sign * 0.5f;
                
                Vec3i64 voxel_pos(
                    static_cast<int64_t>(std::floor(pos.x + offset.x)),
                    static_cast<int64_t>(std::floor(pos.y + offset.y)),
                    static_cast<int64_t>(std::floor(pos.z + offset.z))
                );
                
                auto voxel = world.get_voxel(voxel_pos);
                if (!voxel || !voxel->is_solid()) {
                    normal += offset;
                }
            }
        }
        
        if (length_sq(normal) < 1e-6f) {
            return -incoming;
        }
        
        return normalize(normal);
    }
    
    Color compute_direct_lighting(const VoxelWorld& world, const Vec3f& pos, const Vec3f& normal) {
        const Vec3f light_dir = normalize(Vec3f(0.5f, 1.0f, 0.3f));
        const Color light_color(1.0f, 0.95f, 0.9f);
        
        float ndotl = std::max(0.0f, dot(normal, light_dir));
        
        Vec3f hit_pos;
        Vec3i64 hit_voxel;
        bool shadow = raycast(world, pos, light_dir, 50.0f, hit_pos, hit_voxel);
        
        float shadow_factor = shadow ? 0.3f : 1.0f;
        
        return light_color * ndotl * shadow_factor;
    }
    
    Color compute_ambient_occlusion(const VoxelWorld& world, const Vec3f& pos, const Vec3f& normal) {
        float ao = 0;
        int samples = 0;
        
        for (int i = 0; i < 8; ++i) {
            for (int j = 0; j < 8; ++j) {
                float u = (i + 0.5f) / 8.0f;
                float v = (j + 0.5f) / 8.0f;
                
                Vec3f dir = sample_hemisphere(normal, i * 8 + j);
                
                Vec3f hit_pos;
                Vec3i64 hit_voxel;
                if (!raycast(world, pos, dir, 5.0f, hit_pos, hit_voxel)) {
                    ao += 1.0f;
                }
                samples++;
            }
        }
        
        ao /= static_cast<float>(samples);
        return Color(ao, ao, ao) * 0.5f;
    }
    
    Vec3f sample_hemisphere(const Vec3f& normal, uint32_t seed) const {
        float u = static_cast<float>((seed * 1103515245U) % 100000) / 100000.0f;
        float v = static_cast<float>((seed * 12345U + 67890U) % 100000) / 100000.0f;
        
        Vec3f tangent = std::abs(normal.y) < 0.99f ? Vec3f(0, 1, 0) : Vec3f(1, 0, 0);
        Vec3f bitangent = normalize(cross(tangent, normal));
        Vec3f t = cross(normal, bitangent);
        
        float phi = 2.0f * 3.14159f * u;
        float cos_theta = std::sqrt(v);
        float sin_theta = std::sqrt(1.0f - v);
        
        Vec3f local(
            std::cos(phi) * sin_theta,
            std::sin(phi) * sin_theta,
            cos_theta
        );
        
        return normalize(
            t * local.x +
            bitangent * local.y +
            normal * local.z
        );
    }
    
    Color tone_map(const Color& c) const {
        Color result;
        result.r = c.r / (1.0f + c.r);
        result.g = c.g / (1.0f + c.g);
        result.b = c.b / (1.0f + c.b);
        
        result.r = std::pow(result.r, 1.0f / 2.2f);
        result.g = std::pow(result.g, 1.0f / 2.2f);
        result.b = std::pow(result.b, 1.0f / 2.2f);
        
        return result;
    }
    
    uint32_t width_;
    uint32_t height_;
    std::vector<uint8_t> framebuffer_;
};

struct CameraComponent {
    Vec3f position;
    Vec3f rotation;
    float fov{60.0f};
    float near_clip{0.1f};
    float far_clip{1000.0f};
    
    CameraComponent() = default;
    CameraComponent(const Vec3f& pos, const Vec3f& rot) 
        : position(pos), rotation(rot) {}
};

class CameraSystem : public ecs::System<CameraSystem> {
public:
    static constexpr const char* system_name() { return "CameraSystem"; }
    
    void update_impl(ecs::EntityManager& em, float dt) {}
    
    void move_forward(CameraComponent& cam, float speed) {
        Vec3f dir = get_forward(cam);
        cam.position += dir * speed;
    }
    
    void move_right(CameraComponent& cam, float speed) {
        Vec3f dir = get_right(cam);
        cam.position += dir * speed;
    }
    
    void move_up(CameraComponent& cam, float speed) {
        cam.position.y += speed;
    }
    
    void rotate(CameraComponent& cam, float yaw, float pitch) {
        cam.rotation.x += yaw;
        cam.rotation.y += pitch;
        cam.rotation.y = std::clamp(cam.rotation.y, -1.5f, 1.5f);
    }
    
    Vec3f get_forward(const CameraComponent& cam) const {
        float cy = std::cos(cam.rotation.x);
        float sy = std::sin(cam.rotation.x);
        float cp = std::cos(cam.rotation.y);
        float sp = std::sin(cam.rotation.y);
        
        return Vec3f(cy * cp, sp, sy * cp);
    }
    
    Vec3f get_right(const CameraComponent& cam) const {
        Vec3f forward = get_forward(cam);
        return normalize(cross(Vec3f(0, 1, 0), forward));
    }
};

class RenderSystem : public ecs::System<RenderSystem> {
public:
    static constexpr const char* system_name() { return "RenderSystem"; }
    
    RenderSystem(uint32_t width, uint32_t height) : renderer_(width, height) {}
    
    void update_impl(ecs::EntityManager& em, float dt) {
        auto view = ecs::make_view<CameraComponent, VoxelWorldComponent>(
            em, [this, dt, &em](ecs::EntityId, CameraComponent& cam, VoxelWorldComponent& world) {
                CameraSystem cam_sys;
                Vec3f dir = cam_sys.get_forward(cam);
                
                DDGIComponent* ddgi = nullptr;
                auto ddgi_view = ecs::make_view<DDGIComponent>(
                    em, [&ddgi](ecs::EntityId, DDGIComponent& d) {
                        ddgi = &d;
                    }
                );
                ddgi_view.each();
                
                renderer_.render(*world.world, ddgi, cam.position, dir, cam.fov);
            }
        );
        view.each();
    }
    
    SoftwareRenderer& renderer() { return renderer_; }
    
private:
    SoftwareRenderer renderer_;
};

}
