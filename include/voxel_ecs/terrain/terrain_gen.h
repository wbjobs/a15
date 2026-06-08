#pragma once

#include "../core/math.h"
#include "../voxel/voxel.h"
#include <string>
#include <unordered_map>
#include <vector>
#include <stack>
#include <random>
#include <cmath>
#include <functional>

namespace voxel_ecs::terrain {

using namespace math;
using namespace voxel;

enum class LSymbol : char {
    MOVE_FORWARD = 'F',
    MOVE_WITHOUT_DRAW = 'f',
    TURN_LEFT = '+',
    TURN_RIGHT = '-',
    PITCH_UP = '&',
    PITCH_DOWN = '^',
    ROLL_LEFT = '\\',
    ROLL_RIGHT = '/',
    PUSH_STATE = '[',
    POP_STATE = ']',
    LEAF = 'L',
    FLOWER = 'W',
    THICKEN = '!',
    NARROW = '?',
    REVERSE = '|',
};

struct LSystemParams {
    uint32_t iterations{3};
    float step_length{1.0f};
    float angle{25.0f};
    float initial_thickness{1.0f};
    float thickness_decay{0.8f};
    uint32_t seed{42};
};

struct TurtleState {
    Vec3f position;
    Vec3f direction;
    Vec3f up;
    Vec3f right;
    float thickness;
    uint32_t depth;
};

class LSystem {
public:
    LSystem() {
        set_default_rules();
    }
    
    void set_axiom(const std::string& axiom) {
        axiom_ = axiom;
    }
    
    void add_rule(char predecessor, const std::string& successor, float probability = 1.0f) {
        rules_[predecessor].push_back({successor, probability});
    }
    
    void set_default_rules() {
        axiom_ = "A";
        
        add_rule('A', "F[&A][+A]F[&L]FA", 0.6f);
        add_rule('A', "F[&A][-A]F[^L]FA", 0.3f);
        add_rule('A', "F[+A][&A][-L]F", 0.1f);
        
        add_rule('F', "FF", 0.4f);
        add_rule('F', "F", 0.6f);
    }
    
    std::string generate(uint32_t iterations) const {
        std::string current = axiom_;
        
        std::mt19937 rng(params_.seed);
        std::uniform_real_distribution<float> dist(0.0f, 1.0f);
        
        for (uint32_t i = 0; i < iterations; ++i) {
            std::string next;
            next.reserve(current.size() * 2);
            
            for (char c : current) {
                auto it = rules_.find(c);
                if (it != rules_.end()) {
                    float r = dist(rng);
                    float cumulative = 0.0f;
                    
                    for (const auto& [successor, prob] : it->second) {
                        cumulative += prob;
                        if (r <= cumulative) {
                            next += successor;
                            break;
                        }
                    }
                } else {
                    next += c;
                }
            }
            
            current = std::move(next);
        }
        
        return current;
    }
    
    void set_params(const LSystemParams& params) {
        params_ = params;
    }
    
    const LSystemParams& params() const { return params_; }
    
private:
    std::string axiom_;
    std::unordered_map<char, std::vector<std::pair<std::string, float>>> rules_;
    LSystemParams params_;
};

class TreeGenerator {
public:
    TreeGenerator(VoxelWorld* world) : world_(world) {}
    
    void generate_tree(const Vec3i64& base_pos, const LSystemParams& params, uint32_t type = 0) {
        LSystem lsys;
        lsys.set_params(params);
        
        if (type == 0) {
            lsys.set_axiom("A");
            lsys.add_rule('A', "F[&A][+A][-A]F[^L]FA", 0.5f);
            lsys.add_rule('A', "F[+A][&A]F[L]FA", 0.5f);
            lsys.add_rule('F', "FF", 0.3f);
            lsys.add_rule('F', "F", 0.7f);
        } else if (type == 1) {
            lsys.set_axiom("B");
            lsys.add_rule('B', "F[-B]F[+B]F[^L][&L]B", 0.6f);
            lsys.add_rule('B', "F[\\B][/B]F[L]B", 0.4f);
            lsys.add_rule('F', "F", 1.0f);
        } else {
            lsys.set_axiom("V");
            lsys.add_rule('V', "F[+V][-V]F[L]", 0.7f);
            lsys.add_rule('V', "F[^V][&V]F[L]", 0.3f);
        }
        
        std::string commands = lsys.generate(params.iterations);
        
        TurtleState initial;
        initial.position = Vec3f(
            static_cast<float>(base_pos.x) + 0.5f,
            static_cast<float>(base_pos.y) + 0.5f,
            static_cast<float>(base_pos.z) + 0.5f
        );
        initial.direction = Vec3f(0.0f, 1.0f, 0.0f);
        initial.up = Vec3f(0.0f, 0.0f, 1.0f);
        initial.right = Vec3f(1.0f, 0.0f, 0.0f);
        initial.thickness = params.initial_thickness;
        initial.depth = 0;
        
        execute_commands(commands, initial, params);
    }
    
    void generate_vine(const Vec3i64& start_pos, const Vec3f& grow_dir,
                       uint32_t length, uint32_t seed = 42) {
        std::mt19937 rng(seed);
        std::normal_distribution<float> angle_dist(0.0f, 15.0f);
        std::uniform_real_distribution<float> leaf_dist(0.0f, 1.0f);
        
        Vec3f pos = Vec3f(
            static_cast<float>(start_pos.x) + 0.5f,
            static_cast<float>(start_pos.y) + 0.5f,
            static_cast<float>(start_pos.z) + 0.5f
        );
        
        Vec3f dir = normalize(grow_dir);
        Vec3f up = Vec3f(0.0f, 1.0f, 0.0f);
        
        for (uint32_t i = 0; i < length; ++i) {
            place_vine_segment(pos);
            
            if (leaf_dist(rng) > 0.7f) {
                place_leaf_cluster(pos, up, 2 + (seed % 3));
            }
            
            float yaw = angle_dist(rng) * 3.14159f / 180.0f;
            float pitch = angle_dist(rng) * 3.14159f / 180.0f;
            
            dir = rotate_y(dir, yaw);
            dir = rotate_x(dir, pitch);
            dir = normalize(dir);
            
            pos += dir;
            
            if (!is_air_or_vegetation(pos)) {
                break;
            }
        }
    }
    
    void generate_bush(const Vec3i64& base_pos, float radius, uint32_t seed = 42) {
        std::mt19937 rng(seed);
        std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
        std::uniform_real_distribution<float> height_dist(0.3f, 1.0f);
        
        Vec3f center = Vec3f(
            static_cast<float>(base_pos.x) + 0.5f,
            static_cast<float>(base_pos.y) + 0.5f,
            static_cast<float>(base_pos.z) + 0.5f
        );
        
        uint32_t num_branches = 5 + (seed % 8);
        for (uint32_t i = 0; i < num_branches; ++i) {
            Vec3f dir = Vec3f(dist(rng), height_dist(rng), dist(rng)).normalized();
            float len = radius * (0.5f + std::abs(dist(rng)) * 0.5f);
            
            Vec3f pos = center;
            for (float t = 0; t < len; t += 0.5f) {
                Vec3f branch_pos = center + dir * t;
                place_branch_segment(branch_pos);
            }
            
            Vec3f leaf_center = center + dir * len;
            place_leaf_cluster(leaf_center, Vec3f(0.0f, 1.0f, 0.0f), 3 + (seed % 4));
        }
    }
    
    void populate_jungle(const AABB& bounds, float density = 0.3f, uint32_t seed = 42) {
        std::mt19937 rng(seed);
        std::uniform_real_distribution<float> dist(0.0f, 1.0f);
        std::uniform_int_distribution<int> tree_type_dist(0, 2);
        std::uniform_int_distribution<int> vine_seed_dist(0, 1000);
        
        int64_t min_x = static_cast<int64_t>(std::floor(bounds.min.x));
        int64_t max_x = static_cast<int64_t>(std::ceil(bounds.max.x));
        int64_t min_z = static_cast<int64_t>(std::floor(bounds.min.z));
        int64_t max_z = static_cast<int64_t>(std::ceil(bounds.max.z));
        
        for (int64_t x = min_x; x <= max_x; x += 3) {
            for (int64_t z = min_z; z <= max_z; z += 3) {
                if (dist(rng) > density) continue;
                
                int64_t ground_y = find_ground(x, z, 
                    static_cast<int64_t>(bounds.max.y), 
                    static_cast<int64_t>(bounds.min.y));
                
                if (ground_y >= 0) {
                    Vec3i64 tree_pos(x, ground_y + 1, z);
                    
                    uint32_t type = tree_type_dist(rng);
                    LSystemParams params;
                    
                    if (type == 0) {
                        params.iterations = 4;
                        params.step_length = 1.2f;
                        params.angle = 20.0f;
                        params.initial_thickness = 1.5f;
                        params.seed = vine_seed_dist(rng);
                        generate_tree(tree_pos, params, 0);
                    } else if (type == 1) {
                        params.iterations = 3;
                        params.step_length = 0.8f;
                        params.angle = 30.0f;
                        params.initial_thickness = 1.0f;
                        params.seed = vine_seed_dist(rng);
                        generate_bush(Vec3i64(x, ground_y + 1, z), 3.0f, vine_seed_dist(rng));
                    }
                    
                    if (dist(rng) > 0.5f) {
                        Vec3f vine_dir = Vec3f(dist(rng) - 0.5f, 0.5f, dist(rng) - 0.5f).normalized();
                        generate_vine(Vec3i64(x, ground_y + 2, z), vine_dir, 15 + vine_seed_dist(rng) % 10, vine_seed_dist(rng));
                    }
                }
            }
        }
    }
    
private:
    void execute_commands(const std::string& commands, TurtleState initial, const LSystemParams& params) {
        std::stack<TurtleState> stack;
        TurtleState state = initial;
        
        float angle_rad = params.angle * 3.14159f / 180.0f;
        float step = params.step_length;
        
        for (char cmd : commands) {
            switch (cmd) {
                case 'F': {
                    Vec3f start = state.position;
                    state.position += state.direction * step;
                    draw_line(start, state.position, state.thickness, false);
                    state.thickness *= params.thickness_decay;
                    state.depth++;
                    break;
                }
                case 'f': {
                    state.position += state.direction * step;
                    state.depth++;
                    break;
                }
                case '+': {
                    state.direction = rotate_y(state.direction, -angle_rad);
                    state.right = rotate_y(state.right, -angle_rad);
                    break;
                }
                case '-': {
                    state.direction = rotate_y(state.direction, angle_rad);
                    state.right = rotate_y(state.right, angle_rad);
                    break;
                }
                case '&': {
                    state.direction = rotate_x(state.direction, angle_rad);
                    state.up = rotate_x(state.up, angle_rad);
                    break;
                }
                case '^': {
                    state.direction = rotate_x(state.direction, -angle_rad);
                    state.up = rotate_x(state.up, -angle_rad);
                    break;
                }
                case '\\': {
                    state.direction = rotate_z(state.direction, angle_rad);
                    state.right = rotate_z(state.right, angle_rad);
                    break;
                }
                case '/': {
                    state.direction = rotate_z(state.direction, -angle_rad);
                    state.right = rotate_z(state.right, -angle_rad);
                    break;
                }
                case '[': {
                    stack.push(state);
                    break;
                }
                case ']': {
                    if (!stack.empty()) {
                        state = stack.top();
                        stack.pop();
                    }
                    break;
                }
                case 'L': {
                    place_leaf_cluster(state.position, state.direction, 3);
                    break;
                }
                case 'W': {
                    place_flower(state.position);
                    break;
                }
                case '!': {
                    state.thickness /= params.thickness_decay;
                    break;
                }
                case '?': {
                    state.thickness *= params.thickness_decay;
                    break;
                }
                case '|': {
                    state.direction = -state.direction;
                    break;
                }
                default:
                    break;
            }
        }
    }
    
    void draw_line(const Vec3f& start, const Vec3f& end, float thickness, bool is_leaf) {
        Vec3f diff = end - start;
        float len = length(diff);
        if (len < 0.1f) return;
        
        Vec3f dir = diff / len;
        int32_t steps = static_cast<int32_t>(std::ceil(len));
        
        for (int32_t i = 0; i <= steps; ++i) {
            float t = static_cast<float>(i) / static_cast<float>(steps);
            Vec3f pos = start + dir * len * t;
            
            if (is_leaf) {
                place_leaf(pos);
            } else {
                place_branch_segment(pos, thickness);
            }
        }
    }
    
    void place_branch_segment(const Vec3f& pos, float thickness = 0.8f) {
        int32_t radius = static_cast<int32_t>(std::ceil(thickness * 0.5f));
        
        for (int32_t dx = -radius; dx <= radius; ++dx) {
            for (int32_t dy = -radius; dy <= radius; ++dy) {
                for (int32_t dz = -radius; dz <= radius; ++dz) {
                    float dist = std::sqrt(
                        static_cast<float>(dx * dx) + 
                        static_cast<float>(dy * dy) + 
                        static_cast<float>(dz * dz)
                    );
                    
                    if (dist <= thickness * 0.5f) {
                        Vec3i64 voxel_pos(
                            static_cast<int64_t>(std::floor(pos.x)) + dx,
                            static_cast<int64_t>(std::floor(pos.y)) + dy,
                            static_cast<int64_t>(std::floor(pos.z)) + dz
                        );
                        
                        auto existing = world_->get_voxel(voxel_pos);
                        if (!existing || !existing->is_solid()) {
                            VoxelData wood;
                            wood.albedo = Color(0.35f, 0.2f, 0.1f);
                            wood.flags = VoxelData::FLAG_SOLID | VoxelData::FLAG_VEGETATION;
                            wood.roughness = 0.8f;
                            wood.material_id = 3;
                            world_->set_voxel(voxel_pos, wood);
                        }
                    }
                }
            }
        }
    }
    
    void place_vine_segment(const Vec3f& pos) {
        Vec3i64 voxel_pos(
            static_cast<int64_t>(std::floor(pos.x)),
            static_cast<int64_t>(std::floor(pos.y)),
            static_cast<int64_t>(std::floor(pos.z))
        );
        
        auto existing = world_->get_voxel(voxel_pos);
        if (!existing || !existing->is_solid()) {
            VoxelData vine;
            vine.albedo = Color(0.2f, 0.5f, 0.15f);
            vine.flags = VoxelData::FLAG_SOLID | VoxelData::FLAG_VEGETATION;
            vine.roughness = 0.9f;
            vine.material_id = 5;
            world_->set_voxel(voxel_pos, vine);
        }
    }
    
    void place_leaf(const Vec3f& pos) {
        Vec3i64 voxel_pos(
            static_cast<int64_t>(std::floor(pos.x)),
            static_cast<int64_t>(std::floor(pos.y)),
            static_cast<int64_t>(std::floor(pos.z))
        );
        
        auto existing = world_->get_voxel(voxel_pos);
        if (!existing || !existing->is_solid() || existing->is_vegetation()) {
            VoxelData leaf;
            leaf.albedo = Color(0.15f, 0.45f, 0.1f);
            leaf.flags = VoxelData::FLAG_TRANSPARENT | VoxelData::FLAG_VEGETATION;
            leaf.roughness = 0.7f;
            leaf.material_id = 2;
            world_->set_voxel(voxel_pos, leaf);
        }
    }
    
    void place_leaf_cluster(const Vec3f& center, const Vec3f& normal, uint32_t count = 5) {
        uint32_t seed = static_cast<uint32_t>(
            static_cast<int32_t>(center.x * 73856093.0f) ^ 
            static_cast<int32_t>(center.y * 19349663.0f) ^ 
            static_cast<int32_t>(center.z * 83492791.0f)
        );
        std::mt19937 rng(seed);
        std::normal_distribution<float> dist(0.0f, 0.8f);
        
        Vec3f up(0.0f, 1.0f, 0.0f);
        Vec3f tangent = normalize(cross(normal, up));
        Vec3f bitangent = normalize(cross(normal, tangent));
        
        for (uint32_t i = 0; i < count; ++i) {
            float u = dist(rng);
            float v = dist(rng);
            float w = dist(rng) * 0.5f;
            
            Vec3f leaf_pos = center + tangent * u + bitangent * v + normal * w;
            place_leaf(leaf_pos);
        }
    }
    
    void place_flower(const Vec3f& pos) {
        Vec3i64 voxel_pos(
            static_cast<int64_t>(std::floor(pos.x)),
            static_cast<int64_t>(std::floor(pos.y)),
            static_cast<int64_t>(std::floor(pos.z))
        );
        
        auto existing = world_->get_voxel(voxel_pos);
        if (!existing || !existing->is_solid() || existing->is_vegetation()) {
            VoxelData flower;
            
            uint32_t seed = static_cast<uint32_t>(
                static_cast<int32_t>(pos.x * 73856093.0f) ^ 
                static_cast<int32_t>(pos.y * 19349663.0f) ^ 
                static_cast<int32_t>(pos.z * 83492791.0f)
            );
            std::mt19937 rng(seed);
            std::uniform_int_distribution<int> color_dist(0, 3);
            
            int color = color_dist(rng);
            if (color == 0) {
                flower.albedo = Color(0.9f, 0.2f, 0.4f);
            } else if (color == 1) {
                flower.albedo = Color(0.9f, 0.8f, 0.2f);
            } else if (color == 2) {
                flower.albedo = Color(0.8f, 0.3f, 0.8f);
            } else {
                flower.albedo = Color(1.0f, 1.0f, 1.0f);
            }
            
            flower.flags = VoxelData::FLAG_TRANSPARENT | VoxelData::FLAG_VEGETATION;
            flower.roughness = 0.6f;
            flower.material_id = 4;
            world_->set_voxel(voxel_pos, flower);
        }
    }
    
    int64_t find_ground(int64_t x, int64_t z, int64_t max_y, int64_t min_y) {
        for (int64_t y = max_y; y >= min_y; --y) {
            Vec3i64 pos(x, y, z);
            auto voxel = world_->get_voxel(pos);
            if (voxel && voxel->is_solid() && !voxel->is_water() && !voxel->is_vegetation()) {
                return y;
            }
        }
        return -1;
    }
    
    bool is_air_or_vegetation(const Vec3f& pos) {
        Vec3i64 voxel_pos(
            static_cast<int64_t>(std::floor(pos.x)),
            static_cast<int64_t>(std::floor(pos.y)),
            static_cast<int64_t>(std::floor(pos.z))
        );
        
        auto voxel = world_->get_voxel(voxel_pos);
        return !voxel || !voxel->is_solid() || voxel->is_vegetation();
    }
    
    static Vec3f rotate_x(const Vec3f& v, float angle) {
        float cos_a = std::cos(angle);
        float sin_a = std::sin(angle);
        return Vec3f(
            v.x,
            v.y * cos_a - v.z * sin_a,
            v.y * sin_a + v.z * cos_a
        );
    }
    
    static Vec3f rotate_y(const Vec3f& v, float angle) {
        float cos_a = std::cos(angle);
        float sin_a = std::sin(angle);
        return Vec3f(
            v.x * cos_a + v.z * sin_a,
            v.y,
            -v.x * sin_a + v.z * cos_a
        );
    }
    
    static Vec3f rotate_z(const Vec3f& v, float angle) {
        float cos_a = std::cos(angle);
        float sin_a = std::sin(angle);
        return Vec3f(
            v.x * cos_a - v.y * sin_a,
            v.x * sin_a + v.y * cos_a,
            v.z
        );
    }
    
    VoxelWorld* world_;
};

struct TerrainComponent {
    TreeGenerator* generator;
};

class TerrainSystem : public ecs::System<TerrainSystem> {
public:
    static constexpr const char* system_name() { return "TerrainSystem"; }
    
    void update_impl(ecs::EntityManager& em, float dt) {
        (void)dt;
        auto view = ecs::make_view<TerrainComponent>(
            em, [](ecs::EntityId, TerrainComponent& terrain) {
            }
        );
        view.each();
    }
};

}
