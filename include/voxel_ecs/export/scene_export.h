#pragma once

#include "../core/math.h"
#include "../voxel/voxel.h"
#include "../ecs/ecs.h"
#include <fstream>
#include <cassert>
#include <string>
#include <vector>
#include <array>
#include <unordered_map>
#include <stdexcept>
#include <cstdint>
#include <cstring>

namespace voxel_ecs::export_ {

using namespace math;
using namespace voxel;

struct MeshVertex {
    Vec3f position;
    Vec3f normal;
    Color color;
    Vec2f uv;
};

struct VoxelMesh {
    std::vector<MeshVertex> vertices;
    std::vector<uint32_t> indices;
    std::vector<std::string> materials;
};

class VoxelMesher {
public:
    VoxelMesh generate_mesh(const VoxelWorld& world, const AABB& bounds) {
        VoxelMesh mesh;
        
        Vec3i64 min_pos(
            static_cast<int64_t>(std::floor(bounds.min.x)),
            static_cast<int64_t>(std::floor(bounds.min.y)),
            static_cast<int64_t>(std::floor(bounds.min.z))
        );
        Vec3i64 max_pos(
            static_cast<int64_t>(std::ceil(bounds.max.x)),
            static_cast<int64_t>(std::ceil(bounds.max.y)),
            static_cast<int64_t>(std::ceil(bounds.max.z))
        );
        
        std::unordered_map<uint64_t, uint32_t> vertex_map;
        
        for (int64_t z = min_pos.z; z < max_pos.z; ++z) {
            for (int64_t y = min_pos.y; y < max_pos.y; ++y) {
                for (int64_t x = min_pos.x; x < max_pos.x; ++x) {
                    Vec3i64 pos(x, y, z);
                    auto voxel = world.get_voxel(pos);
                    if (!voxel || !voxel->is_solid()) continue;
                    
                    add_voxel_faces(world, pos, *voxel, mesh, vertex_map);
                }
            }
        }
        
        return mesh;
    }
    
private:
    enum FaceDirection { POS_X, NEG_X, POS_Y, NEG_Y, POS_Z, NEG_Z };
    
    static const std::array<Vec3f, 6> face_offsets;
    static const std::array<Vec3f, 6> face_normals;
    static const std::array<std::array<uint32_t, 6>, 6> face_indices;
    
    void add_voxel_faces(const VoxelWorld& world, const Vec3i64& pos, const VoxelData& voxel,
                         VoxelMesh& mesh, std::unordered_map<uint64_t, uint32_t>& vertex_map) {
        Vec3f center(
            static_cast<float>(pos.x) + 0.5f,
            static_cast<float>(pos.y) + 0.5f,
            static_cast<float>(pos.z) + 0.5f
        );
        
        static const Vec3i64 face_check[] = {
            {1, 0, 0}, {-1, 0, 0},
            {0, 1, 0}, {0, -1, 0},
            {0, 0, 1}, {0, 0, -1}
        };
        
        for (int face = 0; face < 6; ++face) {
            Vec3i64 neighbor_pos = pos + face_check[face];
            auto neighbor = world.get_voxel(neighbor_pos);
            if (neighbor && neighbor->is_solid()) continue;
            
            add_face(center, voxel, face, mesh, vertex_map);
        }
    }
    
    void add_face(const Vec3f& center, const VoxelData& voxel, int face,
                  VoxelMesh& mesh, std::unordered_map<uint64_t, uint32_t>& vertex_map) {
        static const std::array<Vec3f, 4> face_corners[] = {
            { Vec3f(0.5f, -0.5f, -0.5f), Vec3f(0.5f, -0.5f, 0.5f), Vec3f(0.5f, 0.5f, 0.5f), Vec3f(0.5f, 0.5f, -0.5f) },
            { Vec3f(-0.5f, -0.5f, 0.5f), Vec3f(-0.5f, -0.5f, -0.5f), Vec3f(-0.5f, 0.5f, -0.5f), Vec3f(-0.5f, 0.5f, 0.5f) },
            { Vec3f(-0.5f, 0.5f, -0.5f), Vec3f(0.5f, 0.5f, -0.5f), Vec3f(0.5f, 0.5f, 0.5f), Vec3f(-0.5f, 0.5f, 0.5f) },
            { Vec3f(-0.5f, -0.5f, 0.5f), Vec3f(0.5f, -0.5f, 0.5f), Vec3f(0.5f, -0.5f, -0.5f), Vec3f(-0.5f, -0.5f, -0.5f) },
            { Vec3f(-0.5f, -0.5f, 0.5f), Vec3f(-0.5f, 0.5f, 0.5f), Vec3f(0.5f, 0.5f, 0.5f), Vec3f(0.5f, -0.5f, 0.5f) },
            { Vec3f(0.5f, -0.5f, -0.5f), Vec3f(0.5f, 0.5f, -0.5f), Vec3f(-0.5f, 0.5f, -0.5f), Vec3f(-0.5f, -0.5f, -0.5f) }
        };
        
        static const std::array<Vec2f, 4> uvs = {
            Vec2f(0, 0), Vec2f(1, 0), Vec2f(1, 1), Vec2f(0, 1)
        };
        
        uint32_t base_idx = static_cast<uint32_t>(mesh.vertices.size());
        
        for (int i = 0; i < 4; ++i) {
            MeshVertex v;
            v.position = center + face_corners[face][i];
            v.normal = face_normals[face];
            v.color = voxel.albedo;
            v.uv = uvs[i];
            
            uint64_t key = hash_vertex(v);
            auto it = vertex_map.find(key);
            if (it != vertex_map.end()) {
                mesh.indices.push_back(it->second);
            } else {
                vertex_map[key] = static_cast<uint32_t>(mesh.vertices.size());
                mesh.indices.push_back(static_cast<uint32_t>(mesh.vertices.size()));
                mesh.vertices.push_back(v);
            }
        }
        
        static const uint32_t face_idx[] = {0, 1, 2, 0, 2, 3};
        for (int i = 0; i < 6; ++i) {
            mesh.indices.push_back(base_idx + face_idx[i]);
        }
    }
    
    uint64_t hash_vertex(const MeshVertex& v) {
        uint64_t h = 0;
        h ^= std::bit_cast<uint32_t>(v.position.x) * 0x9e3779b9ULL;
        h ^= std::bit_cast<uint32_t>(v.position.y) * 0x85ebca6bULL;
        h ^= std::bit_cast<uint32_t>(v.position.z) * 0xc2b2ae35ULL;
        h ^= std::bit_cast<uint32_t>(v.normal.x) * 0x27d4eb2fULL;
        h ^= std::bit_cast<uint32_t>(v.normal.y) * 0x7ed55d16ULL;
        h ^= std::bit_cast<uint32_t>(v.normal.z) * 0xd5a79147ULL;
        return h;
    }
};

inline const std::array<Vec3f, 6> VoxelMesher::face_offsets = {{
    {0.5f, 0, 0}, {-0.5f, 0, 0},
    {0, 0.5f, 0}, {0, -0.5f, 0},
    {0, 0, 0.5f}, {0, 0, -0.5f}
}};

inline const std::array<Vec3f, 6> VoxelMesher::face_normals = {{
    {1, 0, 0}, {-1, 0, 0},
    {0, 1, 0}, {0, -1, 0},
    {0, 0, 1}, {0, 0, -1}
}};

inline const std::array<std::array<uint32_t, 6>, 6> VoxelMesher::face_indices = {{
    {{0, 1, 2, 0, 2, 3}},
    {{0, 1, 2, 0, 2, 3}},
    {{0, 1, 2, 0, 2, 3}},
    {{0, 1, 2, 0, 2, 3}},
    {{0, 1, 2, 0, 2, 3}},
    {{0, 1, 2, 0, 2, 3}}
}};

class OBJExporter {
public:
    bool export_scene(const VoxelWorld& world, const std::string& filepath, const AABB& bounds = AABB()) {
        VoxelMesher mesher;
        VoxelMesh mesh;
        
        AABB export_bounds = bounds;
        if (export_bounds.min.x > export_bounds.max.x) {
            export_bounds = compute_world_bounds(world);
        }
        
        mesh = mesher.generate_mesh(world, export_bounds);
        
        return write_obj(filepath, mesh);
    }
    
    AABB compute_world_bounds(const VoxelWorld& world) {
        AABB bounds;
        for (const auto& [pos, chunk] : world.chunks()) {
            Vec3f chunk_min(
                static_cast<float>(pos.x * CHUNK_SIZE),
                static_cast<float>(pos.y * CHUNK_SIZE),
                static_cast<float>(pos.z * CHUNK_SIZE)
            );
            Vec3f chunk_max = chunk_min + Vec3f(
                static_cast<float>(CHUNK_SIZE),
                static_cast<float>(CHUNK_SIZE),
                static_cast<float>(CHUNK_SIZE)
            );
            bounds.expand(chunk_min);
            bounds.expand(chunk_max);
        }
        return bounds;
    }
    
private:
    bool write_obj(const std::string& filepath, const VoxelMesh& mesh) {
        std::ofstream file(filepath);
        if (!file.is_open()) return false;
        
        file << "# Voxel GI Engine OBJ Export\n";
        file << "# Vertices: " << mesh.vertices.size() << "\n";
        file << "# Faces: " << mesh.indices.size() / 3 << "\n\n";
        
        for (const auto& v : mesh.vertices) {
            file << "v " << v.position.x << " " << v.position.y << " " << v.position.z << "\n";
        }
        file << "\n";
        
        for (const auto& v : mesh.vertices) {
            file << "vn " << v.normal.x << " " << v.normal.y << " " << v.normal.z << "\n";
        }
        file << "\n";
        
        for (const auto& v : mesh.vertices) {
            file << "vc " << v.color.r << " " << v.color.g << " " << v.color.b << "\n";
        }
        file << "\n";
        
        for (const auto& v : mesh.vertices) {
            file << "vt " << v.uv.x << " " << v.uv.y << "\n";
        }
        file << "\n";
        
        file << "g VoxelMesh\n";
        file << "usemtl Default\n\n";
        
        for (size_t i = 0; i < mesh.indices.size(); i += 3) {
            uint32_t i0 = mesh.indices[i] + 1;
            uint32_t i1 = mesh.indices[i + 1] + 1;
            uint32_t i2 = mesh.indices[i + 2] + 1;
            file << "f " 
                 << i0 << "/" << i0 << "/" << i0 << " "
                 << i1 << "/" << i1 << "/" << i1 << " "
                 << i2 << "/" << i2 << "/" << i2 << "\n";
        }
        
        write_mtl(filepath + ".mtl");
        
        return true;
    }
    
    void write_mtl(const std::string& filepath) {
        std::ofstream file(filepath);
        if (!file.is_open()) return;
        
        file << "newmtl Default\n";
        file << "Kd 0.8 0.8 0.8\n";
        file << "Ks 0.2 0.2 0.2\n";
        file << "Ns 30\n";
        file << "d 1.0\n";
    }
};

class VOXExporter {
public:
    bool export_scene(const VoxelWorld& world, const std::string& filepath, const AABB& bounds = AABB()) {
        AABB export_bounds = bounds;
        if (export_bounds.min.x > export_bounds.max.x) {
            export_bounds = compute_world_bounds(world);
        }
        
        int32_t sx = static_cast<int32_t>(std::floor(export_bounds.min.x));
        int32_t sy = static_cast<int32_t>(std::floor(export_bounds.min.y));
        int32_t sz = static_cast<int32_t>(std::floor(export_bounds.min.z));
        int32_t ex = static_cast<int32_t>(std::ceil(export_bounds.max.x));
        int32_t ey = static_cast<int32_t>(std::ceil(export_bounds.max.y));
        int32_t ez = static_cast<int32_t>(std::ceil(export_bounds.max.z));
        
        uint32_t size_x = static_cast<uint32_t>(ex - sx);
        uint32_t size_y = static_cast<uint32_t>(ey - sy);
        uint32_t size_z = static_cast<uint32_t>(ez - sz);
        
        size_x = std::min(size_x, 256u);
        size_y = std::min(size_y, 256u);
        size_z = std::min(size_z, 256u);
        
        std::vector<VoxelData> voxels;
        for (int32_t z = sz; z < sz + (int32_t)size_z; ++z) {
            for (int32_t y = sy; y < sy + (int32_t)size_y; ++y) {
                for (int32_t x = sx; x < sx + (int32_t)size_x; ++x) {
                    auto v = world.get_voxel(Vec3i64(x, y, z));
                    if (v && v->is_solid()) {
                        voxels.push_back(*v);
                    } else {
                        VoxelData empty;
                        empty.flags = 0;
                        voxels.push_back(empty);
                    }
                }
            }
        }
        
        return write_vox(filepath, size_x, size_y, size_z, voxels);
    }
    
    AABB compute_world_bounds(const VoxelWorld& world) {
        AABB bounds;
        for (const auto& [pos, chunk] : world.chunks()) {
            Vec3f chunk_min(
                static_cast<float>(pos.x * CHUNK_SIZE),
                static_cast<float>(pos.y * CHUNK_SIZE),
                static_cast<float>(pos.z * CHUNK_SIZE)
            );
            Vec3f chunk_max = chunk_min + Vec3f(
                static_cast<float>(CHUNK_SIZE),
                static_cast<float>(CHUNK_SIZE),
                static_cast<float>(CHUNK_SIZE)
            );
            bounds.expand(chunk_min);
            bounds.expand(chunk_max);
        }
        return bounds;
    }
    
private:
#pragma pack(push, 1)
    struct VOXHeader {
        char magic[4];
        uint32_t version;
    };
    
    struct VOXChunk {
        char id[4];
        uint32_t content_size;
        uint32_t children_size;
    };
#pragma pack(pop)
    
    bool write_vox(const std::string& filepath, uint32_t size_x, uint32_t size_y, uint32_t size_z,
                   const std::vector<VoxelData>& voxels) {
        std::ofstream file(filepath, std::ios::binary);
        if (!file.is_open()) return false;
        
        VOXHeader header = {{'V', 'O', 'X', ' '}, 150};
        file.write(reinterpret_cast<char*>(&header), sizeof(header));
        
        write_chunk(file, "MAIN", nullptr, 0, [&]() {
            write_chunk(file, "PACK", nullptr, 0, [&]() {
                uint32_t num_models = 1;
                file.write(reinterpret_cast<char*>(&num_models), sizeof(num_models));
            });
            
            write_chunk(file, "SIZE", nullptr, 0, [&]() {
                file.write(reinterpret_cast<char*>(&size_x), sizeof(size_x));
                file.write(reinterpret_cast<char*>(&size_y), sizeof(size_y));
                file.write(reinterpret_cast<char*>(&size_z), sizeof(size_z));
            });
            
            std::vector<uint8_t> voxel_data;
            voxel_data.reserve(voxels.size() * 4);
            
            uint8_t palette_index = 1;
            std::unordered_map<uint32_t, uint8_t> color_map;
            
            for (uint32_t z = 0; z < size_z; ++z) {
                for (uint32_t y = 0; y < size_y; ++y) {
                    for (uint32_t x = 0; x < size_x; ++x) {
                        uint32_t idx = (z * size_y + y) * size_x + x;
                        const auto& voxel = voxels[idx];
                        if (voxel.is_solid()) {
                            uint32_t color_key = (voxel.albedo.r8() << 24) | 
                                                 (voxel.albedo.g8() << 16) | 
                                                 (voxel.albedo.b8() << 8) | 0xFF;
                            
                            if (color_map.find(color_key) == color_map.end()) {
                                color_map[color_key] = palette_index++;
                            }
                            
                            voxel_data.push_back(static_cast<uint8_t>(x));
                            voxel_data.push_back(static_cast<uint8_t>(y));
                            voxel_data.push_back(static_cast<uint8_t>(z));
                            voxel_data.push_back(color_map[color_key]);
                        }
                    }
                }
            }
            
            write_chunk(file, "XYZI", nullptr, 0, [&]() {
                uint32_t num_voxels = static_cast<uint32_t>(voxel_data.size() / 4);
                file.write(reinterpret_cast<char*>(&num_voxels), sizeof(num_voxels));
                file.write(reinterpret_cast<char*>(voxel_data.data()), voxel_data.size());
            });
            
            write_chunk(file, "RGBA", nullptr, 0, [&]() {
                std::array<uint8_t, 256 * 4> palette = {};
                palette[0] = 0; palette[1] = 0; palette[2] = 0; palette[3] = 0;
                
                for (const auto& [color, idx] : color_map) {
                    uint32_t pidx = idx * 4;
                    palette[pidx]     = (color >> 24) & 0xFF;
                    palette[pidx + 1] = (color >> 16) & 0xFF;
                    palette[pidx + 2] = (color >> 8) & 0xFF;
                    palette[pidx + 3] = color & 0xFF;
                }
                
                for (uint32_t i = 1; i < 256; ++i) {
                    uint32_t pidx = i * 4;
                    if (palette[pidx] == 0 && palette[pidx + 1] == 0 && palette[pidx + 2] == 0) {
                        palette[pidx] = 128;
                        palette[pidx + 1] = 128;
                        palette[pidx + 2] = 128;
                        palette[pidx + 3] = 255;
                    }
                }
                
                file.write(reinterpret_cast<char*>(palette.data()), palette.size());
            });
        });
        
        return true;
    }
    
    template <typename WriteFunc>
    void write_chunk(std::ofstream& file, const char* id, const void* content, uint32_t content_size, 
                     WriteFunc&& write_children) {
        std::streampos start = file.tellp();
        
        VOXChunk chunk = {};
        std::memcpy(chunk.id, id, 4);
        
        file.write(reinterpret_cast<char*>(&chunk), sizeof(chunk));
        
        if (content && content_size > 0) {
            file.write(reinterpret_cast<const char*>(content), content_size);
        }
        
        std::streampos content_start = file.tellp();
        write_children();
        std::streampos content_end = file.tellp();
        
        uint32_t actual_content_size = content_size;
        uint32_t children_size = static_cast<uint32_t>(content_end - content_start);
        
        file.seekp(start);
        chunk.content_size = actual_content_size;
        chunk.children_size = children_size;
        file.write(reinterpret_cast<char*>(&chunk), sizeof(chunk));
        file.seekp(content_end);
    }
};

class SceneExportSystem : public ecs::System<SceneExportSystem> {
public:
    static constexpr const char* system_name() { return "SceneExportSystem"; }
    
    void update_impl(ecs::EntityManager& em, float dt) {}
    
    bool export_obj(const VoxelWorld& world, const std::string& filepath, const AABB& bounds = AABB()) {
        OBJExporter exporter;
        return exporter.export_scene(world, filepath, bounds);
    }
    
    bool export_vox(const VoxelWorld& world, const std::string& filepath, const AABB& bounds = AABB()) {
        VOXExporter exporter;
        return exporter.export_scene(world, filepath, bounds);
    }
};

}
