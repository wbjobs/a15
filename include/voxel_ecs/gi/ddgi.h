#pragma once

#include "../core/math.h"
#include "../voxel/voxel.h"
#include "../ecs/ecs.h"
#include <vector>
#include <array>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <cmath>
#include <cassert>

namespace voxel_ecs::gi {

using namespace math;
using namespace voxel;

constexpr uint32_t IRRADIANCE_SH_COEFFS = 9;
constexpr uint32_t DEPTH_RESOLUTION = 16;
constexpr uint32_t PROBE_GRID_SIZE = 8;
constexpr float PROBE_SPACING = 4.0f;

struct IrradianceProbe {
    Vec3f position;
    std::array<Color, IRRADIANCE_SH_COEFFS> irradiance;
    std::array<float, DEPTH_RESOLUTION * DEPTH_RESOLUTION> depth;
    std::array<Color, DEPTH_RESOLUTION * DEPTH_RESOLUTION> color;
    Vec3f min_bounds;
    Vec3f max_bounds;
    bool dirty{true};
    bool active{true};
    float energy{1.0f};
    
    static Color evaluate_sh(const std::array<Color, IRRADIANCE_SH_COEFFS>& coeffs, const Vec3f& dir) {
        float x = dir.x, y = dir.y, z = dir.z;
        
        std::array<float, IRRADIANCE_SH_COEFFS> sh = {{
            0.282095f,
            0.488603f * y,
            0.488603f * z,
            0.488603f * x,
            1.092548f * x * y,
            1.092548f * y * z,
            0.315392f * (3.0f * z * z - 1.0f),
            1.092548f * x * z,
            0.546274f * (x * x - y * y)
        }};
        
        Color result(0, 0, 0);
        for (uint32_t i = 0; i < IRRADIANCE_SH_COEFFS; ++i) {
            result += coeffs[i] * sh[i];
        }
        return result;
    }
    
    Color get_irradiance(const Vec3f& direction) const {
        return evaluate_sh(irradiance, normalize(direction));
    }
};

struct DDGIProbeGrid {
    Vec3i64 origin;
    Vec3i64 dimensions;
    float spacing;
    std::vector<IrradianceProbe> probes;
    
    DDGIProbeGrid() = default;
    DDGIProbeGrid(const Vec3i64& origin, const Vec3i64& dims, float spacing)
        : origin(origin), dimensions(dims), spacing(spacing) {
        probes.resize(dims.x * dims.y * dims.z);
        for (int64_t z = 0; z < dims.z; ++z) {
            for (int64_t y = 0; y < dims.y; ++y) {
                for (int64_t x = 0; x < dims.x; ++x) {
                    int64_t idx = (z * dims.y + y) * dims.x + x;
                    probes[idx].position = Vec3f(
                        static_cast<float>(origin.x + x) * spacing,
                        static_cast<float>(origin.y + y) * spacing,
                        static_cast<float>(origin.z + z) * spacing
                    );
                    probes[idx].min_bounds = probes[idx].position - Vec3f(spacing, spacing, spacing);
                    probes[idx].max_bounds = probes[idx].position + Vec3f(spacing, spacing, spacing);
                }
            }
        }
    }
    
    IrradianceProbe& at(int64_t x, int64_t y, int64_t z) {
        return probes[(z * dimensions.y + y) * dimensions.x + x];
    }
    
    const IrradianceProbe& at(int64_t x, int64_t y, int64_t z) const {
        return probes[(z * dimensions.y + y) * dimensions.x + x];
    }
    
    int64_t index(int64_t x, int64_t y, int64_t z) const {
        return (z * dimensions.y + y) * dimensions.x + x;
    }
    
    bool in_bounds(int64_t x, int64_t y, int64_t z) const {
        return x >= 0 && x < dimensions.x && y >= 0 && y < dimensions.y && z >= 0 && z < dimensions.z;
    }
};

class VoxelConeTracer {
public:
    VoxelConeTracer(VoxelWorld* world) : world_(world) {}
    
    Color trace_cone(const Vec3f& origin, const Vec3f& direction, 
                     float max_distance, float cone_angle) const {
        const int NUM_CONES = 6;
        const std::array<Vec3f, NUM_CONES> cone_directions = {{
            {1,0,0}, {-1,0,0},
            {0,1,0}, {0,-1,0},
            {0,0,1}, {0,0,-1}
        }};
        
        Color total(0, 0, 0);
        float total_weight = 0;
        
        for (int i = 0; i < NUM_CONES; ++i) {
            float weight = std::max(0.0f, dot(normalize(direction), cone_directions[i]));
            if (weight > 0.01f) {
                Color c = trace_single_cone(origin, cone_directions[i], max_distance, cone_angle);
                total += c * weight;
                total_weight += weight;
            }
        }
        
        return total_weight > 0 ? total / total_weight : Color(0, 0, 0);
    }
    
    Color trace_single_cone(const Vec3f& origin, const Vec3f& dir, 
                            float max_distance, float cone_angle) const {
        Color accumulator(0, 0, 0);
        float opacity = 0.0f;
        
        float tan_half_angle = std::tan(cone_angle * 0.5f);
        float step_size = 1.0f;
        float distance = step_size;
        
        while (distance < max_distance && opacity < 0.95f) {
            float cone_radius = distance * tan_half_angle;
            float mip_level = std::log2(std::max(1.0f, cone_radius));
            
            Vec3f sample_pos = origin + dir * distance;
            Color sample_color = sample_voxel(sample_pos, static_cast<int>(mip_level));
            
            float sample_opacity = 0.0f;
            if (sample_color.r > 0 || sample_color.g > 0 || sample_color.b > 0) {
                sample_opacity = std::min(1.0f, 1.0f / (cone_radius * cone_radius));
            }
            
            float one_minus_opacity = 1.0f - opacity;
            accumulator += sample_color * sample_opacity * one_minus_opacity;
            opacity += sample_opacity * one_minus_opacity;
            
            step_size = std::max(1.0f, cone_radius * 0.5f);
            distance += step_size;
        }
        
        return accumulator;
    }
    
    Color sample_voxel(const Vec3f& pos, int mip_level) const {
        int64_t step = 1 << std::max(0, mip_level);
        Vec3i64 base_pos(
            static_cast<int64_t>(std::floor(pos.x / step) * step),
            static_cast<int64_t>(std::floor(pos.y / step) * step),
            static_cast<int64_t>(std::floor(pos.z / step) * step)
        );
        
        Color total(0, 0, 0);
        int samples = 0;
        
        for (int64_t z = 0; z < step; z += step / 4 + 1) {
            for (int64_t y = 0; y < step; y += step / 4 + 1) {
                for (int64_t x = 0; x < step; x += step / 4 + 1) {
                    Vec3i64 sample_pos = base_pos + Vec3i64(x, y, z);
                    auto voxel = world_->get_voxel(sample_pos);
                    if (voxel) {
                        total += voxel->albedo * (voxel->emission + 0.2f);
                        samples++;
                    }
                }
            }
        }
        
        return samples > 0 ? total / static_cast<float>(samples) : Color(0, 0, 0);
    }
    
    bool raycast(const Vec3f& origin, const Vec3f& direction, 
                 float max_distance, Vec3f& hit_pos, Vec3i64& hit_voxel) const {
        Vec3f ray_pos = origin;
        float step_size = 0.5f;
        float distance = 0;
        
        while (distance < max_distance) {
            Vec3i64 voxel_pos(
                static_cast<int64_t>(std::floor(ray_pos.x)),
                static_cast<int64_t>(std::floor(ray_pos.y)),
                static_cast<int64_t>(std::floor(ray_pos.z))
            );
            
            auto voxel = world_->get_voxel(voxel_pos);
            if (voxel && voxel->is_solid()) {
                hit_pos = ray_pos;
                hit_voxel = voxel_pos;
                return true;
            }
            
            ray_pos += direction * step_size;
            distance += step_size;
        }
        
        return false;
    }
    
private:
    VoxelWorld* world_;
};

class ProbeUpdateSystem {
public:
    ProbeUpdateSystem(VoxelWorld* world, DDGIProbeGrid* grid)
        : world_(world), grid_(grid), tracer_(world) {}
    
    void update_probe(int64_t x, int64_t y, int64_t z, uint32_t num_rays = 256) {
        if (!grid_->in_bounds(x, y, z)) return;
        
        IrradianceProbe& probe = grid_->at(x, y, z);
        update_probe_irradiance(probe, num_rays);
        update_probe_depth(probe);
        probe.dirty = false;
    }
    
    void update_dirty_probes(uint32_t max_probes_per_frame = 8) {
        uint32_t updated = 0;
        for (int64_t z = 0; z < grid_->dimensions.z && updated < max_probes_per_frame; ++z) {
            for (int64_t y = 0; y < grid_->dimensions.y && updated < max_probes_per_frame; ++y) {
                for (int64_t x = 0; x < grid_->dimensions.x && updated < max_probes_per_frame; ++x) {
                    if (grid_->at(x, y, z).dirty) {
                        update_probe(x, y, z, 128);
                        updated++;
                    }
                }
            }
        }
    }
    
    void mark_dirty_region(const AABB& bounds) {
        for (int64_t z = 0; z < grid_->dimensions.z; ++z) {
            for (int64_t y = 0; y < grid_->dimensions.y; ++y) {
                for (int64_t x = 0; x < grid_->dimensions.x; ++x) {
                    IrradianceProbe& probe = grid_->at(x, y, z);
                    AABB probe_bounds(probe.min_bounds, probe.max_bounds);
                    if (probe_bounds.intersects(bounds)) {
                        probe.dirty = true;
                    }
                }
            }
        }
    }
    
    void update_all_probes(uint32_t num_rays = 256) {
        #pragma omp parallel for collapse(3)
        for (int64_t z = 0; z < grid_->dimensions.z; ++z) {
            for (int64_t y = 0; y < grid_->dimensions.y; ++y) {
                for (int64_t x = 0; x < grid_->dimensions.x; ++x) {
                    update_probe(x, y, z, num_rays);
                }
            }
        }
    }
    
    Color sample_gi(const Vec3f& position, const Vec3f& normal, const Vec3f& view_dir) {
        Vec3f grid_pos(
            (position.x / grid_->spacing) - static_cast<float>(grid_->origin.x),
            (position.y / grid_->spacing) - static_cast<float>(grid_->origin.y),
            (position.z / grid_->spacing) - static_cast<float>(grid_->origin.z)
        );
        
        int64_t x0 = static_cast<int64_t>(std::floor(grid_pos.x));
        int64_t y0 = static_cast<int64_t>(std::floor(grid_pos.y));
        int64_t z0 = static_cast<int64_t>(std::floor(grid_pos.z));
        
        float fx = grid_pos.x - static_cast<float>(x0);
        float fy = grid_pos.y - static_cast<float>(y0);
        float fz = grid_pos.z - static_cast<float>(z0);
        
        Color result(0, 0, 0);
        float total_weight = 0;
        
        for (int dz = 0; dz < 2; ++dz) {
            for (int dy = 0; dy < 2; ++dy) {
                for (int dx = 0; dx < 2; ++dx) {
                    int64_t x = x0 + dx;
                    int64_t y = y0 + dy;
                    int64_t z = z0 + dz;
                    
                    if (!grid_->in_bounds(x, y, z)) continue;
                    
                    float wx = dx ? fx : 1.0f - fx;
                    float wy = dy ? fy : 1.0f - fy;
                    float wz = dz ? fz : 1.0f - fz;
                    float weight = wx * wy * wz;
                    
                    const IrradianceProbe& probe = grid_->at(x, y, z);
                    Color probe_color = probe.get_irradiance(normal);
                    
                    Vec3f to_probe = probe.position - position;
                    float dist = length(to_probe);
                    float visibility = compute_visibility(position, normalize(to_probe), dist, probe);
                    
                    result += probe_color * weight * visibility;
                    total_weight += weight;
                }
            }
        }
        
        if (total_weight > 0) {
            result = result / total_weight;
        }
        
        Color indirect_specular = tracer_.trace_cone(position, reflect(-view_dir, normal), 50.0f, 0.1f);
        result += indirect_specular * 0.3f;
        
        return result;
    }
    
private:
    void update_probe_irradiance(IrradianceProbe& probe, uint32_t num_rays) {
        for (auto& c : probe.irradiance) {
            c = Color(0, 0, 0);
        }
        
        std::array<Vec3f, IRRADIANCE_SH_COEFFS> sh_dirs;
        generate_sh_directions(sh_dirs.data(), IRRADIANCE_SH_COEFFS);
        
        #pragma omp parallel for
        for (int64_t i = 0; i < static_cast<int64_t>(num_rays); ++i) {
            Vec3f dir = generate_hemisphere_direction(static_cast<uint32_t>(i), num_rays);
            Color radiance = trace_probe_ray(probe.position, dir, 100.0f);
            
            for (uint32_t j = 0; j < IRRADIANCE_SH_COEFFS; ++j) {
                float sh = evaluate_sh_basis(j, dir);
                #pragma omp atomic
                probe.irradiance[j].r += radiance.r * sh;
                #pragma omp atomic
                probe.irradiance[j].g += radiance.g * sh;
                #pragma omp atomic
                probe.irradiance[j].b += radiance.b * sh;
            }
        }
        
        float weight = 4.0f * 3.14159f / static_cast<float>(num_rays);
        for (auto& c : probe.irradiance) {
            c = c * weight;
        }
    }
    
    void update_probe_depth(IrradianceProbe& probe) {
        for (uint32_t i = 0; i < DEPTH_RESOLUTION * DEPTH_RESOLUTION; ++i) {
            uint32_t u = i % DEPTH_RESOLUTION;
            uint32_t v = i / DEPTH_RESOLUTION;
            
            Vec3f dir = generate_octahedron_direction(u, v, DEPTH_RESOLUTION);
            float depth = trace_depth(probe.position, dir, 100.0f);
            
            probe.depth[i] = depth;
            
            if (depth < 100.0f) {
                Vec3f hit_pos = probe.position + dir * depth;
                Vec3i64 voxel_pos(
                    static_cast<int64_t>(std::floor(hit_pos.x)),
                    static_cast<int64_t>(std::floor(hit_pos.y)),
                    static_cast<int64_t>(std::floor(hit_pos.z))
                );
                auto voxel = world_->get_voxel(voxel_pos);
                probe.color[i] = voxel ? voxel->albedo : Color(0, 0, 0);
            } else {
                probe.color[i] = Color(0.1f, 0.1f, 0.2f);
            }
        }
    }
    
    Color trace_probe_ray(const Vec3f& origin, const Vec3f& dir, float max_dist) const {
        Color result(0, 0, 0);
        Color throughput(1, 1, 1);
        Vec3f pos = origin;
        Vec3f current_dir = dir;
        float remaining_dist = max_dist;
        
        for (int bounce = 0; bounce < 3; ++bounce) {
            Vec3f hit_pos;
            Vec3i64 hit_voxel;
            
            if (!tracer_.raycast(pos, current_dir, remaining_dist, hit_pos, hit_voxel)) {
                result += throughput * Color(0.1f, 0.1f, 0.2f);
                break;
            }
            
            auto voxel = world_->get_voxel(hit_voxel);
            if (!voxel) break;
            
            result += throughput * voxel->albedo * voxel->emission;
            
            if (bounce == 2) break;
            
            throughput = throughput * voxel->albedo * 0.5f;
            
            Vec3f normal = compute_normal(hit_pos, current_dir);
            current_dir = sample_hemisphere(normal, static_cast<uint32_t>(bounce * 137 + 7));
            
            float hit_dist = length(hit_pos - pos);
            remaining_dist -= hit_dist;
            pos = hit_pos + normal * 0.01f;
        }
        
        return result;
    }
    
    float trace_depth(const Vec3f& origin, const Vec3f& dir, float max_dist) const {
        Vec3f hit_pos;
        Vec3i64 hit_voxel;
        if (tracer_.raycast(origin, dir, max_dist, hit_pos, hit_voxel)) {
            return length(hit_pos - origin);
        }
        return max_dist;
    }
    
    float compute_visibility(const Vec3f& pos, const Vec3f& dir, float dist, const IrradianceProbe& probe) const {
        (void)probe;
        Vec3f hit_pos;
        Vec3i64 hit_voxel;
        if (tracer_.raycast(pos, dir, dist * 0.9f, hit_pos, hit_voxel)) {
            return 0.0f;
        }
        return 1.0f;
    }
    
    Vec3f compute_normal(const Vec3f& pos, const Vec3f& incoming) const {
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
                
                auto voxel = world_->get_voxel(voxel_pos);
                if (!voxel || !voxel->is_solid()) {
                    normal += offset;
                }
            }
        }
        
        if (length_sq(normal) < 1e-6f) {
            return Vec3f(-incoming.x, -incoming.y, -incoming.z);
        }
        
        return normalize(normal);
    }
    
    float evaluate_sh_basis(uint32_t idx, const Vec3f& dir) const {
        float x = dir.x, y = dir.y, z = dir.z;
        switch (idx) {
            case 0: return 0.282095f;
            case 1: return 0.488603f * y;
            case 2: return 0.488603f * z;
            case 3: return 0.488603f * x;
            case 4: return 1.092548f * x * y;
            case 5: return 1.092548f * y * z;
            case 6: return 0.315392f * (3.0f * z * z - 1.0f);
            case 7: return 1.092548f * x * z;
            case 8: return 0.546274f * (x * x - y * y);
            default: return 0;
        }
    }
    
    void generate_sh_directions(Vec3f* dirs, uint32_t count) const {
        for (uint32_t i = 0; i < count; ++i) {
            float u = static_cast<float>(i) / static_cast<float>(count);
            float v = static_cast<float>(((i * 2654435761U) % 10000) / 10000.0f);
            dirs[i] = generate_hemisphere_direction(u, v);
        }
    }
    
    Vec3f generate_hemisphere_direction(uint32_t sample_idx, uint32_t total_samples) const {
        float u = static_cast<float>(sample_idx) / static_cast<float>(total_samples);
        float v = static_cast<float>(((sample_idx * 1103515245U + 12345U) % 100000) / 100000.0f);
        return generate_hemisphere_direction(u, v);
    }
    
    Vec3f generate_hemisphere_direction(float u, float v) const {
        float phi = 2.0f * 3.14159f * u;
        float cos_theta = std::sqrt(1.0f - v);
        float sin_theta = std::sqrt(v);
        
        return Vec3f(
            std::cos(phi) * sin_theta,
            cos_theta,
            std::sin(phi) * sin_theta
        );
    }
    
    Vec3f generate_octahedron_direction(uint32_t u, uint32_t v, uint32_t resolution) const {
        float fu = (static_cast<float>(u) + 0.5f) / static_cast<float>(resolution) * 2.0f - 1.0f;
        float fv = (static_cast<float>(v) + 0.5f) / static_cast<float>(resolution) * 2.0f - 1.0f;
        
        Vec3f p(fu, fv, 1.0f - std::abs(fu) - std::abs(fv));
        
        if (p.z < 0.0f) {
            float px = p.x;
            p.x = (1.0f - std::abs(p.y)) * (px >= 0.0f ? 1.0f : -1.0f);
            p.y = (1.0f - std::abs(px)) * (p.y >= 0.0f ? 1.0f : -1.0f);
        }
        
        return normalize(p);
    }
    
    Vec3f sample_hemisphere(const Vec3f& normal, uint32_t seed) const {
        float u = static_cast<float>((seed * 1103515245U) % 100000) / 100000.0f;
        float v = static_cast<float>((seed * 12345U + 67890U) % 100000) / 100000.0f;
        
        Vec3f tangent = std::abs(normal.y) < 0.99f ? Vec3f(0, 1, 0) : Vec3f(1, 0, 0);
        Vec3f bitangent = normalize(cross(tangent, normal));
        tangent = cross(normal, bitangent);
        
        float phi = 2.0f * 3.14159f * u;
        float cos_theta = std::sqrt(v);
        float sin_theta = std::sqrt(1.0f - v);
        
        Vec3f local(
            std::cos(phi) * sin_theta,
            std::sin(phi) * sin_theta,
            cos_theta
        );
        
        return normalize(
            tangent * local.x +
            bitangent * local.y +
            normal * local.z
        );
    }
    
    Vec3f reflect(const Vec3f& v, const Vec3f& n) const {
        return v - n * 2.0f * dot(v, n);
    }
    
    VoxelWorld* world_;
    DDGIProbeGrid* grid_;
    VoxelConeTracer tracer_;
};

struct DDGIComponent {
    DDGIProbeGrid* grid;
    ProbeUpdateSystem* updater;
};

class DDGISystem : public ecs::System<DDGISystem> {
public:
    static constexpr const char* system_name() { return "DDGISystem"; }
    
    void update_impl(ecs::EntityManager& em, float dt) {
        auto view = ecs::make_view<DDGIComponent, VoxelWorldComponent>(
            em, [this, dt](ecs::EntityId, DDGIComponent& ddgi, VoxelWorldComponent& world) {
                auto dirty = world.world->get_dirty_chunks();
                for (const auto& chunk_pos : dirty) {
                    Vec3f world_min(
                        static_cast<float>(chunk_pos.x * CHUNK_SIZE),
                        static_cast<float>(chunk_pos.y * CHUNK_SIZE),
                        static_cast<float>(chunk_pos.z * CHUNK_SIZE)
                    );
                    Vec3f world_max = world_min + Vec3f(
                        static_cast<float>(CHUNK_SIZE),
                        static_cast<float>(CHUNK_SIZE),
                        static_cast<float>(CHUNK_SIZE)
                    );
                    ddgi.updater->mark_dirty_region(AABB(world_min, world_max));
                }
                world.world->clear_dirty_chunks();
                
                ddgi.updater->update_dirty_probes(max_probes_per_frame_);
            }
        );
        view.each();
    }
    
    void set_max_probes_per_frame(uint32_t max) { max_probes_per_frame_ = max; }
    
private:
    uint32_t max_probes_per_frame_{8};
};

class GISystem {
public:
    Color compute_indirect_lighting(const Vec3f& position, const Vec3f& normal, 
                                     const Vec3f& view_dir, const DDGIComponent& ddgi) {
        return ddgi.updater->sample_gi(position, normal, view_dir);
    }
};

}
