#pragma once

#include "../core/math.h"
#include "../ecs/ecs.h"
#include <cstdint>
#include <vector>
#include <unordered_map>
#include <memory>
#include <array>
#include <optional>
#include <mutex>
#include <shared_mutex>
#include <cassert>
#include <iostream>
#include <iomanip>

namespace voxel_ecs::voxel {

using namespace math;

constexpr int64_t CHUNK_SIZE = 32;
constexpr int64_t BRICK_SIZE = 8;
constexpr uint32_t MAX_LOD = 6;

struct VoxelData {
    Color albedo{1.0f, 1.0f, 1.0f};
    float roughness{0.5f};
    float metallic{0.0f};
    float emission{0.0f};
    uint8_t flags{0};
    
    static constexpr uint8_t FLAG_SOLID = 1 << 0;
    static constexpr uint8_t FLAG_TRANSPARENT = 1 << 1;
    
    bool is_solid() const { return (flags & FLAG_SOLID) != 0; }
    bool is_transparent() const { return (flags & FLAG_TRANSPARENT) != 0; }
};

struct alignas(8) SVDAGNode {
    uint32_t child_mask{0};
    uint32_t data_index{0};
    uint32_t children[8]{0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
                        0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};
    bool is_leaf{false};
    
    bool has_child(uint8_t idx) const { return (child_mask & (1 << idx)) != 0; }
    uint8_t child_count() const { return std::popcount(child_mask); }
};

struct SVDAGData {
    Color color;
    float opacity{1.0f};
};

class SVDAG {
public:
    SVDAG() {
        nodes_.reserve(1024);
        data_.reserve(1024);
        nodes_.push_back(SVDAGNode{});
    }
    
    void insert(const Vec3i64& pos, const VoxelData& voxel, uint32_t depth = MAX_LOD) {
        insert_recursive(0, pos, voxel, depth, Vec3i64(0, 0, 0), 1 << (MAX_LOD - 1));
    }
    
    std::optional<VoxelData> query(const Vec3i64& pos, uint32_t depth = MAX_LOD) const {
        return query_recursive(0, pos, depth, Vec3i64(0, 0, 0), 1 << (MAX_LOD - 1));
    }
    
    void remove(const Vec3i64& pos, uint32_t depth = MAX_LOD) {
        remove_recursive(0, pos, depth, Vec3i64(0, 0, 0), 1 << (MAX_LOD - 1));
    }
    
    void optimize() {
        hash_map_.clear();
        compacted_nodes_.clear();
        compacted_data_.clear();
        
        uint32_t new_root = compact_node(0);
        (void)new_root;
        assert(new_root == 0);
        
        nodes_.swap(compacted_nodes_);
        data_.swap(compacted_data_);
    }
    
    const std::vector<SVDAGNode>& nodes() const { return nodes_; }
    const std::vector<SVDAGData>& data() const { return data_; }
    
    size_t memory_usage() const {
        return nodes_.capacity() * sizeof(SVDAGNode) + 
               data_.capacity() * sizeof(SVDAGData) +
               hash_map_.size() * (sizeof(uint64_t) + sizeof(uint32_t));
    }
    
    void clear() {
        nodes_.clear();
        data_.clear();
        nodes_.push_back(SVDAGNode{});
    }
    
private:
    uint8_t get_child_index(const Vec3i64& pos, const Vec3i64& center, int64_t half_size) const {
        (void)half_size;
        uint8_t idx = 0;
        if (pos.x >= center.x) idx |= 1;
        if (pos.y >= center.y) idx |= 2;
        if (pos.z >= center.z) idx |= 4;
        return idx;
    }
    
    Vec3i64 get_child_center(const Vec3i64& center, int64_t half_size, uint8_t idx) const {
        int64_t quarter = half_size / 2;
        Vec3i64 offset(
            (idx & 1) ? quarter : -quarter,
            (idx & 2) ? quarter : -quarter,
            (idx & 4) ? quarter : -quarter
        );
        return center + offset;
    }
    
    uint32_t allocate_node() {
        uint32_t idx = static_cast<uint32_t>(nodes_.size());
        nodes_.push_back(SVDAGNode{});
        return idx;
    }
    
    uint32_t allocate_data(const SVDAGData& d) {
        uint32_t idx = static_cast<uint32_t>(data_.size());
        data_.push_back(d);
        return idx;
    }
    
    void insert_recursive(uint32_t node_idx, const Vec3i64& pos, const VoxelData& voxel,
                         uint32_t depth, const Vec3i64& center, int64_t half_size) {
        if (depth == 0) {
            SVDAGNode& node = nodes_[node_idx];
            node.is_leaf = true;
            node.data_index = allocate_data(SVDAGData{voxel.albedo, 1.0f});
            node.child_mask = voxel.is_solid() ? 0xFF : 0;
            return;
        }
        
        uint8_t child_idx = get_child_index(pos, center, half_size);
        int64_t quarter = half_size / 2;
        
        bool has_child = nodes_[node_idx].has_child(child_idx);
        uint32_t actual_child;
        
        if (!has_child) {
            uint32_t new_node = allocate_node();
            nodes_[node_idx].children[child_idx] = new_node;
            nodes_[node_idx].child_mask |= (1 << child_idx);
            actual_child = new_node;
        } else {
            actual_child = nodes_[node_idx].children[child_idx];
        }
        
        Vec3i64 child_center = get_child_center(center, half_size, child_idx);
        insert_recursive(actual_child, pos, voxel, depth - 1, child_center, quarter);
    }
    
    std::optional<VoxelData> query_recursive(uint32_t node_idx, const Vec3i64& pos,
                                            uint32_t depth, const Vec3i64& center, int64_t half_size) const {
        const SVDAGNode& node = nodes_[node_idx];
        
        if (node.is_leaf) {
            if (node.child_mask) {
                VoxelData v;
                v.albedo = data_[node.data_index].color;
                v.flags = VoxelData::FLAG_SOLID;
                return v;
            }
            return std::nullopt;
        }
        
        uint8_t child_idx = get_child_index(pos, center, half_size);
        if (!node.has_child(child_idx)) {
            return std::nullopt;
        }
        
        uint32_t actual_child = node.children[child_idx];
        Vec3i64 child_center = get_child_center(center, half_size, child_idx);
        return query_recursive(actual_child, pos, depth - 1, child_center, half_size / 2);
    }
    
    void remove_recursive(uint32_t node_idx, const Vec3i64& pos,
                         uint32_t depth, const Vec3i64& center, int64_t half_size) {
        SVDAGNode& node = nodes_[node_idx];
        
        if (depth == 0) {
            node.child_mask = 0;
            node.is_leaf = false;
            return;
        }
        
        uint8_t child_idx = get_child_index(pos, center, half_size);
        if (!node.has_child(child_idx)) return;
        
        uint32_t actual_child = node.children[child_idx];
        Vec3i64 child_center = get_child_center(center, half_size, child_idx);
        remove_recursive(actual_child, pos, depth - 1, child_center, half_size / 2);
        
        const SVDAGNode& child = nodes_[actual_child];
        if (child.child_mask == 0 && !child.is_leaf) {
            node.child_mask &= ~(1 << child_idx);
            node.children[child_idx] = 0xFFFFFFFF;
        }
    }
    
    uint64_t hash_node(uint32_t node_idx) {
        const SVDAGNode& node = nodes_[node_idx];
        uint64_t h = node.child_mask | (static_cast<uint64_t>(node.is_leaf) << 8);
        
        if (node.is_leaf) {
            const SVDAGData& d = data_[node.data_index];
            h ^= std::bit_cast<uint32_t>(d.color.r) * 0x9e3779b9ULL;
            h ^= std::bit_cast<uint32_t>(d.color.g) * 0x85ebca6bULL;
            h ^= std::bit_cast<uint32_t>(d.color.b) * 0xc2b2ae35ULL;
        } else {
            for (uint8_t i = 0; i < 8; ++i) {
                if (node.has_child(i)) {
                    h ^= hash_node(node.children[i]) * (0x9e3779b9ULL << i);
                }
            }
        }
        
        return h;
    }
    
    uint32_t compact_node(uint32_t old_idx) {
        uint64_t h = hash_node(old_idx);
        auto it = hash_map_.find(h);
        if (it != hash_map_.end()) {
            return it->second;
        }
        
        uint32_t new_idx = static_cast<uint32_t>(compacted_nodes_.size());
        
        SVDAGNode old_node = nodes_[old_idx];
        SVDAGNode new_node = old_node;
        
        compacted_nodes_.push_back(new_node);
        
        hash_map_[h] = new_idx;
        
        if (old_node.is_leaf) {
            new_node.data_index = static_cast<uint32_t>(compacted_data_.size());
            compacted_data_.push_back(data_[old_node.data_index]);
        } else {
            for (uint8_t i = 0; i < 8; ++i) {
                if (old_node.has_child(i)) {
                    new_node.children[i] = compact_node(old_node.children[i]);
                }
            }
        }
        
        compacted_nodes_[new_idx] = new_node;
        
        return new_idx;
    }
    
    std::vector<SVDAGNode> nodes_;
    std::vector<SVDAGData> data_;
    std::unordered_map<uint64_t, uint32_t> hash_map_;
    std::vector<SVDAGNode> compacted_nodes_;
    std::vector<SVDAGData> compacted_data_;
};

struct VoxelChunk {
    Vec3i64 position;
    SVDAG svdag;
    bool modified{false};
    uint64_t last_modified{0};
    
    bool is_empty() const {
        return svdag.nodes().size() <= 1;
    }
};

struct Vec3i64Hash {
    size_t operator()(const Vec3i64& v) const noexcept {
        return static_cast<size_t>(hash_vec(v));
    }
};

class VoxelWorld {
public:
    VoxelWorld() = default;
    
    void set_voxel(const Vec3i64& world_pos, const VoxelData& voxel) {
        std::unique_lock lock(chunks_mutex_);
        Vec3i64 chunk_pos = floor_div(world_pos, CHUNK_SIZE);
        Vec3i64 local_pos = world_pos - chunk_pos * CHUNK_SIZE;
        
        auto& chunk = get_or_create_chunk(chunk_pos);
        chunk.svdag.insert(local_pos, voxel);
        chunk.modified = true;
        chunk.last_modified = ++modification_counter_;
        
        mark_dirty_chunks(chunk_pos);
    }
    
    std::optional<VoxelData> get_voxel(const Vec3i64& world_pos) const {
        std::shared_lock lock(chunks_mutex_);
        Vec3i64 chunk_pos = floor_div(world_pos, CHUNK_SIZE);
        Vec3i64 local_pos = world_pos - chunk_pos * CHUNK_SIZE;
        
        auto it = chunks_.find(chunk_pos);
        if (it == chunks_.end()) return std::nullopt;
        
        return it->second.svdag.query(local_pos);
    }
    
    void remove_voxel(const Vec3i64& world_pos) {
        std::unique_lock lock(chunks_mutex_);
        Vec3i64 chunk_pos = floor_div(world_pos, CHUNK_SIZE);
        Vec3i64 local_pos = world_pos - chunk_pos * CHUNK_SIZE;
        
        auto it = chunks_.find(chunk_pos);
        if (it == chunks_.end()) return;
        
        it->second.svdag.remove(local_pos);
        it->second.modified = true;
        it->second.last_modified = ++modification_counter_;
        
        mark_dirty_chunks(chunk_pos);
        
        if (it->second.is_empty()) {
            chunks_.erase(it);
        }
    }
    
    VoxelChunk& get_or_create_chunk(const Vec3i64& chunk_pos) {
        auto it = chunks_.find(chunk_pos);
        if (it == chunks_.end()) {
            it = chunks_.emplace(chunk_pos, VoxelChunk{chunk_pos, SVDAG{}}).first;
        }
        return it->second;
    }
    
    const VoxelChunk* get_chunk(const Vec3i64& chunk_pos) const {
        auto it = chunks_.find(chunk_pos);
        return it != chunks_.end() ? &it->second : nullptr;
    }
    
    void optimize_all() {
        std::unique_lock lock(chunks_mutex_);
        for (auto& [pos, chunk] : chunks_) {
            if (chunk.modified) {
                chunk.svdag.optimize();
                chunk.modified = false;
            }
        }
    }
    
    const std::unordered_map<Vec3i64, VoxelChunk, Vec3i64Hash>& chunks() const { return chunks_; }
    
    std::vector<Vec3i64> get_dirty_chunks() const {
        std::shared_lock lock(chunks_mutex_);
        return dirty_chunks_;
    }
    
    void clear_dirty_chunks() {
        std::unique_lock lock(chunks_mutex_);
        dirty_chunks_.clear();
    }
    
    size_t total_memory() const {
        std::shared_lock lock(chunks_mutex_);
        size_t total = 0;
        for (const auto& [pos, chunk] : chunks_) {
            total += chunk.svdag.memory_usage();
        }
        return total;
    }
    
    void clear() {
        std::unique_lock lock(chunks_mutex_);
        chunks_.clear();
        dirty_chunks_.clear();
        modification_counter_ = 0;
    }
    
private:
    void mark_dirty_chunks(const Vec3i64& center) {
        static const Vec3i64 offsets[] = {
            {0,0,0}, {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}
        };
        
        for (const auto& off : offsets) {
            Vec3i64 pos = center + off;
            if (std::find(dirty_chunks_.begin(), dirty_chunks_.end(), pos) == dirty_chunks_.end()) {
                dirty_chunks_.push_back(pos);
            }
        }
    }
    
    std::unordered_map<Vec3i64, VoxelChunk, Vec3i64Hash> chunks_;
    std::vector<Vec3i64> dirty_chunks_;
    mutable std::shared_mutex chunks_mutex_;
    uint64_t modification_counter_{0};
};

struct VoxelWorldComponent {
    VoxelWorld* world;
};

struct VoxelEditEvent {
    enum class Type { ADD, REMOVE };
    Type type;
    Vec3i64 position;
    VoxelData data;
};

class VoxelEditSystem : public ecs::System<VoxelEditSystem> {
public:
    static constexpr const char* system_name() { return "VoxelEditSystem"; }
    
    void add_edit(const VoxelEditEvent& event) {
        edits_.push_back(event);
    }
    
    void update_impl(ecs::EntityManager& em, float dt) {
        auto view = ecs::make_view<VoxelWorldComponent>(
            em, [this](ecs::EntityId, VoxelWorldComponent& world_comp) {
                process_edits(*world_comp.world);
            }
        );
        view.each();
    }
    
    const std::vector<VoxelEditEvent>& edits() const { return edits_; }
    
private:
    void process_edits(VoxelWorld& world) {
        for (const auto& edit : edits_) {
            switch (edit.type) {
                case VoxelEditEvent::Type::ADD:
                    world.set_voxel(edit.position, edit.data);
                    break;
                case VoxelEditEvent::Type::REMOVE:
                    world.remove_voxel(edit.position);
                    break;
            }
        }
        edits_.clear();
    }
    
    std::vector<VoxelEditEvent> edits_;
};

}
