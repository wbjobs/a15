#pragma once

#include "../core/math.h"
#include <vector>
#include <unordered_map>
#include <array>
#include <memory>
#include <cmath>
#include <functional>

namespace voxel_ecs::material {

using namespace math;

struct PBRMaterial {
    Color albedo{1.0f, 1.0f, 1.0f};
    float roughness{0.5f};
    float metallic{0.0f};
    float emissive{0.0f};
    Color emissive_color{0.0f, 0.0f, 0.0f};
    
    float normal_strength{1.0f};
    float ao{1.0f};
    float displacement{0.0f};
    float opacity{1.0f};
    
    float ior{1.45f};
    float transmission{0.0f};
    float clearcoat{0.0f};
    float clearcoat_roughness{0.0f};
    
    float sheen{0.0f};
    float sheen_roughness{0.5f};
    Color sheen_color{1.0f, 1.0f, 1.0f};
    
    uint32_t id{0};
    std::string name;
};

template<uint32_t Size = 256>
class Texture2D {
public:
    Texture2D() : data_(Size * Size, Color(0, 0, 0)) {}
    
    Color& operator()(uint32_t x, uint32_t y) {
        x = x % Size;
        y = y % Size;
        return data_[y * Size + x];
    }
    
    const Color& operator()(uint32_t x, uint32_t y) const {
        x = x % Size;
        y = y % Size;
        return data_[y * Size + x];
    }
    
    Color sample(float u, float v) const {
        u = std::fmod(u, 1.0f);
        v = std::fmod(v, 1.0f);
        if (u < 0) u += 1.0f;
        if (v < 0) v += 1.0f;
        
        float x = u * static_cast<float>(Size - 1);
        float y = v * static_cast<float>(Size - 1);
        
        uint32_t x0 = static_cast<uint32_t>(std::floor(x));
        uint32_t y0 = static_cast<uint32_t>(std::floor(y));
        uint32_t x1 = (x0 + 1) % Size;
        uint32_t y1 = (y0 + 1) % Size;
        
        float fx = x - static_cast<float>(x0);
        float fy = y - static_cast<float>(y0);
        
        Color c00 = (*this)(x0, y0);
        Color c10 = (*this)(x1, y0);
        Color c01 = (*this)(x0, y1);
        Color c11 = (*this)(x1, y1);
        
        Color c0 = c00 * (1.0f - fx) + c10 * fx;
        Color c1 = c01 * (1.0f - fx) + c11 * fx;
        
        return c0 * (1.0f - fy) + c1 * fy;
    }
    
    void fill(const Color& color) {
        std::fill(data_.begin(), data_.end(), color);
    }
    
    uint32_t size() const { return Size; }
    
    const std::vector<Color>& data() const { return data_; }
    
private:
    std::vector<Color> data_;
};

template<uint32_t Size = 256>
class HeightMap {
public:
    HeightMap() : data_(Size * Size, 0.0f) {}
    
    float& operator()(uint32_t x, uint32_t y) {
        x = x % Size;
        y = y % Size;
        return data_[y * Size + x];
    }
    
    const float& operator()(uint32_t x, uint32_t y) const {
        x = x % Size;
        y = y % Size;
        return data_[y * Size + x];
    }
    
    float sample(float u, float v) const {
        u = std::fmod(u, 1.0f);
        v = std::fmod(v, 1.0f);
        if (u < 0) u += 1.0f;
        if (v < 0) v += 1.0f;
        
        float x = u * static_cast<float>(Size - 1);
        float y = v * static_cast<float>(Size - 1);
        
        uint32_t x0 = static_cast<uint32_t>(std::floor(x));
        uint32_t y0 = static_cast<uint32_t>(std::floor(y));
        uint32_t x1 = (x0 + 1) % Size;
        uint32_t y1 = (y0 + 1) % Size;
        
        float fx = x - static_cast<float>(x0);
        float fy = y - static_cast<float>(y0);
        
        float h00 = (*this)(x0, y0);
        float h10 = (*this)(x1, y0);
        float h01 = (*this)(x0, y1);
        float h11 = (*this)(x1, y1);
        
        float h0 = h00 * (1.0f - fx) + h10 * fx;
        float h1 = h01 * (1.0f - fx) + h11 * fx;
        
        return h0 * (1.0f - fy) + h1 * fy;
    }
    
    Vec3f sample_normal(float u, float v, float scale = 1.0f) const {
        float eps = 1.0f / static_cast<float>(Size);
        float h_l = sample(u - eps, v);
        float h_r = sample(u + eps, v);
        float h_d = sample(u, v - eps);
        float h_u = sample(u, v + eps);
        
        Vec3f normal(
            h_l - h_r,
            2.0f * scale * eps,
            h_d - h_u
        );
        
        return normalize(normal);
    }
    
    void fill(float value) {
        std::fill(data_.begin(), data_.end(), value);
    }
    
    uint32_t size() const { return Size; }
    
private:
    std::vector<float> data_;
};

template<uint32_t Size = 256>
class TextureSet {
public:
    Texture2D<Size> albedo;
    Texture2D<Size> normal;
    Texture2D<Size> roughness;
    Texture2D<Size> metallic;
    Texture2D<Size> ao;
    Texture2D<Size> emissive;
    HeightMap<Size> height;
    
    void set_from_material(const PBRMaterial& mat) {
        albedo.fill(mat.albedo);
        normal.fill(Color(0.5f, 0.5f, 1.0f));
        roughness.fill(Color(mat.roughness, mat.roughness, mat.roughness));
        metallic.fill(Color(mat.metallic, mat.metallic, mat.metallic));
        ao.fill(Color(mat.ao, mat.ao, mat.ao));
        emissive.fill(mat.emissive_color * mat.emissive);
        height.fill(mat.displacement);
    }
    
    PBRMaterial sample_material(float u, float v) const {
        PBRMaterial mat;
        
        Color alb = albedo.sample(u, v);
        mat.albedo = alb;
        
        Color norm = normal.sample(u, v);
        mat.normal_strength = 1.0f;
        
        Color rough = roughness.sample(u, v);
        mat.roughness = rough.r;
        
        Color met = metallic.sample(u, v);
        mat.metallic = met.r;
        
        Color ambient = ao.sample(u, v);
        mat.ao = ambient.r;
        
        Color emi = emissive.sample(u, v);
        mat.emissive_color = emi;
        mat.emissive = (emi.r + emi.g + emi.b) / 3.0f;
        
        mat.displacement = height.sample(u, v);
        
        return mat;
    }
};

class MaterialLibrary {
public:
    uint32_t add_material(const PBRMaterial& mat) {
        uint32_t id = static_cast<uint32_t>(materials_.size());
        PBRMaterial new_mat = mat;
        new_mat.id = id;
        materials_.push_back(new_mat);
        
        auto tex_set = std::make_unique<TextureSet<256>>();
        tex_set->set_from_material(new_mat);
        textures_[id] = std::move(tex_set);
        
        return id;
    }
    
    const PBRMaterial& get_material(uint32_t id) const {
        return materials_.at(id);
    }
    
    PBRMaterial& get_material(uint32_t id) {
        return materials_.at(id);
    }
    
    TextureSet<256>* get_textures(uint32_t id) {
        auto it = textures_.find(id);
        if (it != textures_.end()) {
            return it->second.get();
        }
        return nullptr;
    }
    
    void generate_procedural_texture(uint32_t material_id, 
        const std::function<void(TextureSet<256>&, uint32_t, uint32_t)>& generator) {
        auto* tex = get_textures(material_id);
        if (!tex) return;
        
        for (uint32_t y = 0; y < 256; ++y) {
            for (uint32_t x = 0; x < 256; ++x) {
                generator(*tex, x, y);
            }
        }
    }
    
    uint32_t count() const { return materials_.size(); }
    
private:
    std::vector<PBRMaterial> materials_;
    std::unordered_map<uint32_t, std::unique_ptr<TextureSet<256>>> textures_;
};

class TransitionTextureGenerator {
public:
    static void generate_blend(TextureSet<256>& result,
                               const TextureSet<256>& mat_a,
                               const TextureSet<256>& mat_b,
                               const std::function<float(float, float)>& blend_func) {
        for (uint32_t y = 0; y < 256; ++y) {
            for (uint32_t x = 0; x < 256; ++x) {
                float u = static_cast<float>(x) / 255.0f;
                float v = static_cast<float>(y) / 255.0f;
                float blend = std::clamp(blend_func(u, v), 0.0f, 1.0f);
                
                Color ca = mat_a.albedo(x, y);
                Color cb = mat_b.albedo(x, y);
                result.albedo(x, y) = ca * (1.0f - blend) + cb * blend;
                
                Color na = mat_a.normal(x, y);
                Color nb = mat_b.normal(x, y);
                result.normal(x, y) = na * (1.0f - blend) + nb * blend;
                
                Color ra = mat_a.roughness(x, y);
                Color rb = mat_b.roughness(x, y);
                result.roughness(x, y) = ra * (1.0f - blend) + rb * blend;
                
                Color ma = mat_a.metallic(x, y);
                Color mb = mat_b.metallic(x, y);
                result.metallic(x, y) = ma * (1.0f - blend) + mb * blend;
                
                Color aa = mat_a.ao(x, y);
                Color ab = mat_b.ao(x, y);
                result.ao(x, y) = aa * (1.0f - blend) + ab * blend;
                
                Color ea = mat_a.emissive(x, y);
                Color eb = mat_b.emissive(x, y);
                result.emissive(x, y) = ea * (1.0f - blend) + eb * blend;
                
                float ha = mat_a.height(x, y);
                float hb = mat_b.height(x, y);
                result.height(x, y) = ha * (1.0f - blend) + hb * blend;
            }
        }
    }
    
    static void generate_noise_blend(TextureSet<256>& result,
                                     const TextureSet<256>& mat_a,
                                     const TextureSet<256>& mat_b,
                                     float scale = 8.0f, float threshold = 0.5f, float falloff = 0.1f) {
        generate_blend(result, mat_a, mat_b, 
            [scale, threshold, falloff](float u, float v) -> float {
                float noise = value_noise(u * scale, v * scale);
                noise = noise * 0.5f + 0.5f;
                
                float edge0 = threshold - falloff;
                float edge1 = threshold + falloff;
                return smoothstep(edge0, edge1, noise);
            }
        );
    }
    
    static void generate_gradient_blend(TextureSet<256>& result,
                                        const TextureSet<256>& mat_a,
                                        const TextureSet<256>& mat_b,
                                        const Vec2f& direction = Vec2f(1.0f, 0.0f),
                                        float offset = 0.0f) {
        Vec2f dir = direction.normalized();
        generate_blend(result, mat_a, mat_b,
            [dir, offset](float u, float v) -> float {
                Vec2f uv(u - 0.5f, v - 0.5f);
                float t = dot(uv, dir) + offset;
                return std::clamp(t + 0.5f, 0.0f, 1.0f);
            }
        );
    }
    
    static void generate_voronoi_blend(TextureSet<256>& result,
                                       const TextureSet<256>& mat_a,
                                       const TextureSet<256>& mat_b,
                                       uint32_t cell_count = 8, float randomness = 0.5f) {
        generate_blend(result, mat_a, mat_b,
            [cell_count, randomness](float u, float v) -> float {
                return voronoi(u * cell_count, v * cell_count, randomness);
            }
        );
    }
    
private:
    static float value_noise(float x, float y) {
        int32_t xi = static_cast<int32_t>(std::floor(x));
        int32_t yi = static_cast<int32_t>(std::floor(y));
        float xf = x - static_cast<float>(xi);
        float yf = y - static_cast<float>(yi);
        
        xf = xf * xf * (3.0f - 2.0f * xf);
        yf = yf * yf * (3.0f - 2.0f * yf);
        
        float v00 = hash(xi, yi);
        float v10 = hash(xi + 1, yi);
        float v01 = hash(xi, yi + 1);
        float v11 = hash(xi + 1, yi + 1);
        
        float v0 = v00 * (1.0f - xf) + v10 * xf;
        float v1 = v01 * (1.0f - xf) + v11 * xf;
        
        return v0 * (1.0f - yf) + v1 * yf;
    }
    
    static float hash(int32_t x, int32_t y) {
        int32_t h = x * 374761393 + y * 668265263;
        h = (h ^ (h >> 13)) * 1274126177;
        h = h ^ (h >> 16);
        return static_cast<float>(h & 0xFFFF) / 65535.0f * 2.0f - 1.0f;
    }
    
    static float smoothstep(float edge0, float edge1, float x) {
        float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    }
    
    static float voronoi(float x, float y, float randomness) {
        int32_t xi = static_cast<int32_t>(std::floor(x));
        int32_t yi = static_cast<int32_t>(std::floor(y));
        
        float min_dist = 1e10f;
        
        for (int32_t dy = -1; dy <= 1; ++dy) {
            for (int32_t dx = -1; dx <= 1; ++dx) {
                float px = static_cast<float>(xi + dx) + hash2(xi + dx, yi + dy, 0) * randomness;
                float py = static_cast<float>(yi + dy) + hash2(xi + dx, yi + dy, 1) * randomness;
                
                float dist = (x - px) * (x - px) + (y - py) * (y - py);
                min_dist = std::min(min_dist, dist);
            }
        }
        
        return std::sqrt(min_dist);
    }
    
    static float hash2(int32_t x, int32_t y, int32_t z) {
        int32_t h = x * 374761393 + y * 668265263 + z * 2147483647;
        h = (h ^ (h >> 13)) * 1274126177;
        h = h ^ (h >> 16);
        return static_cast<float>(h & 0xFFFF) / 65535.0f;
    }
};

class PBRRenderer {
public:
    static Color compute_lighting(const PBRMaterial& mat,
                                  const Vec3f& normal,
                                  const Vec3f& view_dir,
                                  const Vec3f& light_dir,
                                  const Color& light_color,
                                  float light_intensity) {
        Vec3f n = normalize(normal);
        Vec3f v = normalize(-view_dir);
        Vec3f l = normalize(light_dir);
        Vec3f h = normalize(v + l);
        
        float ndotl = std::max(0.0f, dot(n, l));
        float ndotv = std::max(0.0f, dot(n, v));
        float ndoth = std::max(0.0f, dot(n, h));
        float vdoth = std::max(0.0f, dot(v, h));
        
        Color F0(0.04f, 0.04f, 0.04f);
        if (mat.metallic > 0.5f) {
            F0 = mat.albedo * mat.metallic;
        }
        
        Color F = fresnel_schlick(vdoth, F0);
        float D = ggx_distribution(ndoth, mat.roughness);
        float G = geometry_smith(ndotl, ndotv, mat.roughness);
        
        float denom = 4.0f * ndotl * ndotv + 1e-4f;
        Color specular = (F * D * G) / denom;
        
        Color diffuse = mat.albedo * (1.0f - mat.metallic) / 3.14159f;
        
        Color kd = Color(1.0f, 1.0f, 1.0f) - F;
        kd = kd * (1.0f - mat.metallic);
        
        Color result = (kd * diffuse + specular) * light_color * light_intensity * ndotl;
        
        if (mat.emissive > 0.0f) {
            result += mat.emissive_color * mat.emissive;
        }
        
        result = result * mat.ao;
        
        return result;
    }
    
private:
    static Color fresnel_schlick(float cos_theta, const Color& f0) {
        return f0 + (Color(1.0f, 1.0f, 1.0f) - f0) * std::pow(1.0f - cos_theta, 5.0f);
    }
    
    static float ggx_distribution(float ndoth, float roughness) {
        float a = roughness * roughness;
        float a2 = a * a;
        float denom = ndoth * ndoth * (a2 - 1.0f) + 1.0f;
        return a2 / (3.14159f * denom * denom + 1e-4f);
    }
    
    static float geometry_schlick_ggx(float ndotv, float k) {
        return ndotv / (ndotv * (1.0f - k) + k + 1e-4f);
    }
    
    static float geometry_smith(float ndotl, float ndotv, float roughness) {
        float r = roughness + 1.0f;
        float k = (r * r) / 8.0f;
        return geometry_schlick_ggx(ndotl, k) * geometry_schlick_ggx(ndotv, k);
    }
};

struct MaterialComponent {
    MaterialLibrary* library;
};

class MaterialSystem : public ecs::System<MaterialSystem> {
public:
    static constexpr const char* system_name() { return "MaterialSystem"; }
    
    void update_impl(ecs::EntityManager& em, float dt) {
        (void)dt;
        auto view = ecs::make_view<MaterialComponent>(
            em, [](ecs::EntityId, MaterialComponent& mat) {
            }
        );
        view.each();
    }
};

}
