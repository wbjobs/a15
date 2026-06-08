#pragma once

#include "../core/math.h"
#include "../voxel/voxel.h"
#include <vector>
#include <cmath>
#include <array>

namespace voxel_ecs::water {

using namespace math;
using namespace voxel;

struct GerstnerWave {
    Vec2f direction;
    float amplitude;
    float frequency;
    float phase;
    float steepness;
};

class WaterSimulation {
public:
    WaterSimulation(VoxelWorld* world, int64_t water_level = 10)
        : world_(world), water_level_(water_level) {
        init_default_waves();
    }
    
    void init_default_waves() {
        waves_.push_back({Vec2f(1.0f, 0.0f).normalized(), 0.3f, 0.5f, 0.0f, 0.5f});
        waves_.push_back({Vec2f(0.7f, 0.7f).normalized(), 0.2f, 0.8f, 1.0f, 0.4f});
        waves_.push_back({Vec2f(-0.5f, 0.5f).normalized(), 0.15f, 1.2f, 2.0f, 0.3f});
        waves_.push_back({Vec2f(0.3f, -0.8f).normalized(), 0.1f, 1.5f, 3.0f, 0.2f});
        waves_.push_back({Vec2f(-0.8f, -0.3f).normalized(), 0.08f, 2.0f, 4.0f, 0.15f});
        waves_.push_back({Vec2f(0.6f, 0.2f).normalized(), 0.05f, 2.5f, 5.0f, 0.1f});
    }
    
    void add_wave(const GerstnerWave& wave) {
        waves_.push_back(wave);
    }
    
    float get_water_height(float x, float z, float time) const {
        float height = static_cast<float>(water_level_);
        Vec2f pos(x, z);
        
        for (const auto& wave : waves_) {
            float dot_prod = dot(wave.direction, pos);
            float phase = wave.frequency * dot_prod + wave.phase * time;
            
            height += wave.amplitude * std::sin(phase);
        }
        
        return height;
    }
    
    Vec3f get_water_normal(float x, float z, float time) const {
        float eps = 0.01f;
        float h0 = get_water_height(x, z, time);
        float hx = get_water_height(x + eps, z, time);
        float hz = get_water_height(x, z + eps, time);
        
        Vec3f normal(
            h0 - hx,
            eps,
            h0 - hz
        );
        
        return normalize(normal);
    }
    
    Vec3f compute_refraction(const Vec3f& view_dir, const Vec3f& normal, float ior = 1.33f) const {
        float eta = 1.0f / ior;
        float cos_i = dot(-view_dir, normal);
        
        if (cos_i < 0.0f) {
            return view_dir;
        }
        
        float sin_t_sq = eta * eta * (1.0f - cos_i * cos_i);
        
        if (sin_t_sq > 1.0f) {
            float r0 = (1.0f - ior) / (1.0f + ior);
            r0 *= r0;
            float reflectance = r0 + (1.0f - r0) * std::pow(1.0f - cos_i, 5.0f);
            if (reflectance > 0.8f) {
                return reflect(view_dir, normal);
            }
        }
        
        float cos_t = std::sqrt(std::max(0.0f, 1.0f - sin_t_sq));
        return eta * view_dir + (eta * cos_i - cos_t) * normal;
    }
    
    float compute_caustic_intensity(const Vec3f& world_pos, float time) const {
        Vec2f pos(world_pos.x, world_pos.z);
        float intensity = 0.0f;
        
        for (const auto& wave : waves_) {
            float dot_prod = dot(wave.direction, pos);
            float phase = wave.frequency * dot_prod + wave.phase * time;
            
            float dx = -wave.direction.y * wave.amplitude * wave.frequency * wave.steepness * std::cos(phase);
            float dz = wave.direction.x * wave.amplitude * wave.frequency * wave.steepness * std::cos(phase);
            
            float jacobian = std::abs(1.0f + dx) * std::abs(1.0f + dz);
            intensity += wave.amplitude * (1.0f / std::max(0.1f, jacobian));
        }
        
        return std::clamp(intensity * 0.5f, 0.0f, 2.0f);
    }
    
    void update_water_surface(const AABB& bounds, float time) {
        int64_t min_x = static_cast<int64_t>(std::floor(bounds.min.x));
        int64_t max_x = static_cast<int64_t>(std::ceil(bounds.max.x));
        int64_t min_z = static_cast<int64_t>(std::floor(bounds.min.z));
        int64_t max_z = static_cast<int64_t>(std::ceil(bounds.max.z));
        
        for (int64_t x = min_x; x <= max_x; ++x) {
            for (int64_t z = min_z; z <= max_z; ++z) {
                float height = get_water_height(
                    static_cast<float>(x) + 0.5f,
                    static_cast<float>(z) + 0.5f,
                    time
                );
                
                int64_t base_y = water_level_;
                int64_t surface_y = static_cast<int64_t>(std::floor(height));
                
                for (int64_t y = base_y - 2; y <= surface_y; ++y) {
                    Vec3i64 pos(x, y, z);
                    auto existing = world_->get_voxel(pos);
                    
                    if (!existing || !existing->is_water()) {
                        VoxelData water;
                        water.albedo = Color(0.2f, 0.4f, 0.8f);
                        water.flags = VoxelData::FLAG_WATER | VoxelData::FLAG_TRANSPARENT;
                        water.roughness = 0.1f;
                        water.metallic = 0.1f;
                        world_->set_voxel(pos, water);
                    }
                }
                
                for (int64_t y = surface_y + 1; y <= base_y + 2; ++y) {
                    Vec3i64 pos(x, y, z);
                    auto existing = world_->get_voxel(pos);
                    if (existing && existing->is_water()) {
                        world_->remove_voxel(pos);
                    }
                }
            }
        }
    }
    
    Color sample_water_color(const Vec3f& position, const Vec3f& view_dir, 
                             const Vec3f& light_dir, float time) const {
        Vec3f normal = get_water_normal(position.x, position.z, time);
        
        float fresnel = std::pow(1.0f - std::max(0.0f, dot(-view_dir, normal)), 3.0f);
        fresnel = std::clamp(0.02f + 0.98f * fresnel, 0.0f, 1.0f);
        
        Vec3f refracted = compute_refraction(view_dir, normal);
        float depth = compute_water_depth(position);
        
        Color deep_color(0.05f, 0.1f, 0.3f);
        Color shallow_color(0.3f, 0.6f, 0.9f);
        float depth_factor = std::clamp(depth / 5.0f, 0.0f, 1.0f);
        Color water_color = shallow_color * (1.0f - depth_factor) + deep_color * depth_factor;
        
        Color sky_color(0.5f, 0.7f, 0.9f);
        Color reflection_color = sky_color * fresnel;
        
        float specular = std::pow(std::max(0.0f, dot(reflect(-light_dir, normal), -view_dir)), 64.0f);
        Color specular_color(1.0f, 1.0f, 1.0f);
        
        return water_color * (1.0f - fresnel) + reflection_color + specular_color * specular * 0.5f;
    }
    
    float compute_water_depth(const Vec3f& position) const {
        int64_t start_y = static_cast<int64_t>(std::floor(position.y));
        
        for (int64_t y = start_y; y >= water_level_ - 20; --y) {
            Vec3i64 voxel_pos(
                static_cast<int64_t>(std::floor(position.x)),
                y,
                static_cast<int64_t>(std::floor(position.z))
            );
            
            auto voxel = world_->get_voxel(voxel_pos);
            if (!voxel || !voxel->is_water()) {
                return position.y - static_cast<float>(y);
            }
        }
        
        return 20.0f;
    }
    
    int64_t water_level() const { return water_level_; }
    void set_water_level(int64_t level) { water_level_ = level; }
    
    const std::vector<GerstnerWave>& waves() const { return waves_; }
    
private:
    VoxelWorld* world_;
    int64_t water_level_;
    std::vector<GerstnerWave> waves_;
};

struct WaterComponent {
    WaterSimulation* simulation;
    float time{0.0f};
    float time_scale{1.0f};
};

class WaterSystem : public ecs::System<WaterSystem> {
public:
    static constexpr const char* system_name() { return "WaterSystem"; }
    
    void update_impl(ecs::EntityManager& em, float dt) {
        auto view = ecs::make_view<WaterComponent, VoxelWorldComponent>(
            em, [dt](ecs::EntityId, WaterComponent& water, VoxelWorldComponent& world) {
                water.time += dt * water.time_scale;
                
                AABB bounds(
                    Vec3f(-100.0f, static_cast<float>(water.simulation->water_level()) - 5.0f, -100.0f),
                    Vec3f(100.0f, static_cast<float>(water.simulation->water_level()) + 5.0f, 100.0f)
                );
                
                water.simulation->update_water_surface(bounds, water.time);
            }
        );
        view.each();
    }
};

}
