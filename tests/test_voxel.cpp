#include <voxel_ecs/voxel/voxel.h>
#include <voxel_ecs/gi/ddgi.h>
#include <voxel_ecs/export/scene_export.h>
#include <iostream>
#include <cassert>
#include <chrono>

using namespace voxel_ecs;
using namespace voxel_ecs::voxel;
using namespace voxel_ecs::math;
using namespace voxel_ecs::gi;
using namespace voxel_ecs::export_;

int main() {
    std::cout << "=== Voxel and GI Tests ===" << std::endl;
    
    {
        std::cout << "Test 1: VoxelWorld basic operations..." << std::endl;
        VoxelWorld world;
        
        VoxelData v1;
        v1.albedo = Color(1, 0, 0);
        v1.flags = VoxelData::FLAG_SOLID;
        world.set_voxel(Vec3i64(0, 0, 0), v1);
        
        VoxelData v2;
        v2.albedo = Color(0, 1, 0);
        v2.flags = VoxelData::FLAG_SOLID;
        world.set_voxel(Vec3i64(1, 2, 3), v2);
        
        auto got1 = world.get_voxel(Vec3i64(0, 0, 0));
        assert(got1);
        assert(got1->albedo.r == 1.0f);
        assert(got1->is_solid());
        
        auto got2 = world.get_voxel(Vec3i64(1, 2, 3));
        assert(got2);
        assert(got2->albedo.g == 1.0f);
        
        auto got3 = world.get_voxel(Vec3i64(100, 100, 100));
        assert(!got3);
        
        world.remove_voxel(Vec3i64(0, 0, 0));
        auto got4 = world.get_voxel(Vec3i64(0, 0, 0));
        assert(!got4);
        
        std::cout << "  PASSED" << std::endl;
    }
    
    {
        std::cout << "Test 2: Large coordinate support (64-bit)..." << std::endl;
        VoxelWorld world;
        
        int64_t big = 1LL << 30;
        
        VoxelData v;
        v.albedo = Color(1, 1, 1);
        v.flags = VoxelData::FLAG_SOLID;
        
        world.set_voxel(Vec3i64(big, big, big), v);
        world.set_voxel(Vec3i64(-big, -big, -big), v);
        
        auto got1 = world.get_voxel(Vec3i64(big, big, big));
        auto got2 = world.get_voxel(Vec3i64(-big, -big, -big));
        
        assert(got1 && got1->is_solid());
        assert(got2 && got2->is_solid());
        
        std::cout << "  PASSED" << std::endl;
    }
    
    {
        std::cout << "Test 3: Chunk management..." << std::endl;
        VoxelWorld world;
        
        VoxelData v;
        v.albedo = Color(1, 0, 0);
        v.flags = VoxelData::FLAG_SOLID;
        
        for (int x = 0; x < 100; ++x) {
            for (int z = 0; z < 100; ++z) {
                world.set_voxel(Vec3i64(x, 0, z), v);
            }
        }
        
        size_t num_chunks = world.chunks().size();
        std::cout << "  Created " << num_chunks << " chunks for 100x100 area" << std::endl;
        assert(num_chunks > 0);
        
        std::cout << "  Memory usage: " << world.total_memory() / 1024 << " KB" << std::endl;
        
        std::cout << "  PASSED" << std::endl;
    }
    
    {
        std::cout << "Test 4: SVDAG insert and query..." << std::endl;
        SVDAG svdag;
        
        VoxelData v;
        v.albedo = Color(1, 0, 0);
        v.flags = VoxelData::FLAG_SOLID;
        
        for (int x = 0; x < 8; ++x) {
            for (int y = 0; y < 8; ++y) {
                for (int z = 0; z < 8; ++z) {
                    if ((x + y + z) % 2 == 0) {
                        svdag.insert(Vec3i64(x, y, z), v);
                    }
                }
            }
        }
        
        int count = 0;
        for (int x = 0; x < 8; ++x) {
            for (int y = 0; y < 8; ++y) {
                for (int z = 0; z < 8; ++z) {
                    auto result = svdag.query(Vec3i64(x, y, z));
                    if ((x + y + z) % 2 == 0) {
                        assert(result);
                        count++;
                    } else {
                        assert(!result);
                    }
                }
            }
        }
        
        std::cout << "  Inserted and queried " << count << " voxels" << std::endl;
        std::cout << "  SVDAG nodes: " << svdag.nodes().size() << std::endl;
        std::cout << "  SVDAG memory: " << svdag.memory_usage() / 1024 << " KB" << std::endl;
        
        std::cout << "  PASSED" << std::endl;
    }
    
    {
        std::cout << "Test 5: SVDAG remove and optimize..." << std::endl;
        SVDAG svdag;
        
        VoxelData v;
        v.albedo = Color(1, 1, 1);
        v.flags = VoxelData::FLAG_SOLID;
        
        for (int x = 0; x < 4; ++x) {
            for (int y = 0; y < 4; ++y) {
                for (int z = 0; z < 4; ++z) {
                    svdag.insert(Vec3i64(x, y, z), v);
                }
            }
        }
        
        size_t nodes_before = svdag.nodes().size();
        size_t mem_before = svdag.memory_usage();
        
        svdag.optimize();
        
        size_t nodes_after = svdag.nodes().size();
        size_t mem_after = svdag.memory_usage();
        
        std::cout << "  Before optimize: " << nodes_before << " nodes, " << mem_before << " bytes" << std::endl;
        std::cout << "  After optimize: " << nodes_after << " nodes, " << mem_after << " bytes" << std::endl;
        
        for (int i = 0; i < 10; ++i) {
            svdag.insert(Vec3i64(i, i, i), v);
        }
        
        for (int i = 0; i < 5; ++i) {
            svdag.remove(Vec3i64(i, i, i));
        }
        
        for (int i = 5; i < 10; ++i) {
            assert(svdag.query(Vec3i64(i, i, i)));
        }
        for (int i = 0; i < 5; ++i) {
            assert(!svdag.query(Vec3i64(i, i, i)));
        }
        
        std::cout << "  PASSED" << std::endl;
    }
    
    {
        std::cout << "Test 6: DDGI probe grid creation..." << std::endl;
        VoxelWorld world;
        
        VoxelData v;
        v.albedo = Color(1, 0.5f, 0.2f);
        v.flags = VoxelData::FLAG_SOLID;
        for (int x = 0; x < 10; ++x) {
            for (int z = 0; z < 10; ++z) {
                world.set_voxel(Vec3i64(x, 0, z), v);
            }
        }
        
        DDGIProbeGrid grid(Vec3i64(0, 0, 0), Vec3i64(4, 3, 4), 4.0f);
        
        assert(grid.dimensions.x == 4);
        assert(grid.dimensions.y == 3);
        assert(grid.dimensions.z == 4);
        assert(grid.probes.size() == 48);
        
        assert(grid.at(0, 0, 0).position.x == 0.0f);
        assert(grid.at(1, 0, 0).position.x == 4.0f);
        
        assert(!grid.in_bounds(4, 0, 0));
        assert(!grid.in_bounds(0, 3, 0));
        assert(!grid.in_bounds(0, 0, 4));
        
        std::cout << "  PASSED" << std::endl;
    }
    
    {
        std::cout << "Test 7: VoxelConeTracer basic raycast..." << std::endl;
        VoxelWorld world;
        
        VoxelData v;
        v.albedo = Color(1, 1, 1);
        v.flags = VoxelData::FLAG_SOLID;
        world.set_voxel(Vec3i64(5, 5, 5), v);
        
        VoxelConeTracer tracer(&world);
        
        Vec3f hit_pos;
        Vec3i64 hit_voxel;
        
        bool hit = tracer.raycast(
            Vec3f(0, 5, 5),
            Vec3f(1, 0, 0),
            10.0f,
            hit_pos,
            hit_voxel
        );
        
        assert(hit);
        assert(hit_voxel.x == 5);
        assert(hit_voxel.y == 5);
        assert(hit_voxel.z == 5);
        
        bool miss = tracer.raycast(
            Vec3f(0, 5, 5),
            Vec3f(0, 1, 0),
            10.0f,
            hit_pos,
            hit_voxel
        );
        
        assert(!miss);
        
        std::cout << "  PASSED" << std::endl;
    }
    
    {
        std::cout << "Test 8: ProbeUpdateSystem..." << std::endl;
        VoxelWorld world;
        
        VoxelData ground;
        ground.albedo = Color(0.3f, 0.7f, 0.2f);
        ground.flags = VoxelData::FLAG_SOLID;
        for (int x = -20; x < 20; ++x) {
            for (int z = -20; z < 20; ++z) {
                world.set_voxel(Vec3i64(x, 0, z), ground);
            }
        }
        
        VoxelData light;
        light.albedo = Color(1, 1, 0.8f);
        light.emission = 3.0f;
        light.flags = VoxelData::FLAG_SOLID;
        world.set_voxel(Vec3i64(0, 10, 0), light);
        
        DDGIProbeGrid grid(Vec3i64(0, 0, 0), Vec3i64(3, 2, 3), 4.0f);
        ProbeUpdateSystem updater(&world, &grid);
        
        std::cout << "  Updating probes (quick update)..." << std::endl;
        auto start = std::chrono::high_resolution_clock::now();
        updater.update_probe(1, 1, 1, 64);
        auto end = std::chrono::high_resolution_clock::now();
        double time = std::chrono::duration<double>(end - start).count();
        
        std::cout << "  Probe update time: " << time * 1000 << " ms" << std::endl;
        
        const auto& probe = grid.at(1, 1, 1);
        Color irr = probe.get_irradiance(Vec3f(0, -1, 0));
        
        std::cout << "  Irradiance at probe, looking down: (" 
                  << irr.r << ", " << irr.g << ", " << irr.b << ")" << std::endl;
        
        assert(irr.r > 0);
        assert(irr.g > 0);
        assert(irr.b > 0);
        
        std::cout << "  PASSED" << std::endl;
    }
    
    {
        std::cout << "Test 9: VoxelMesher..." << std::endl;
        VoxelWorld world;
        
        VoxelData v;
        v.albedo = Color(1, 0, 0);
        v.flags = VoxelData::FLAG_SOLID;
        
        for (int x = 0; x < 3; ++x) {
            for (int y = 0; y < 3; ++y) {
                for (int z = 0; z < 3; ++z) {
                    world.set_voxel(Vec3i64(x, y, z), v);
                }
            }
        }
        
        VoxelMesher mesher;
        AABB bounds(Vec3f(-1, -1, -1), Vec3f(4, 4, 4));
        VoxelMesh mesh = mesher.generate_mesh(world, bounds);
        
        std::cout << "  Generated mesh: " << mesh.vertices.size() << " vertices, " 
                  << mesh.indices.size() << " indices" << std::endl;
        
        assert(mesh.vertices.size() > 0);
        assert(mesh.indices.size() > 0);
        
        std::cout << "  PASSED" << std::endl;
    }
    
    {
        std::cout << "Test 10: Scene export (OBJ and VOX)..." << std::endl;
        VoxelWorld world;
        
        VoxelData v1;
        v1.albedo = Color(1, 0, 0);
        v1.flags = VoxelData::FLAG_SOLID;
        world.set_voxel(Vec3i64(0, 0, 0), v1);
        
        VoxelData v2;
        v2.albedo = Color(0, 1, 0);
        v2.flags = VoxelData::FLAG_SOLID;
        world.set_voxel(Vec3i64(1, 0, 0), v2);
        
        VoxelData v3;
        v3.albedo = Color(0, 0, 1);
        v3.flags = VoxelData::FLAG_SOLID;
        world.set_voxel(Vec3i64(2, 0, 0), v3);
        
        SceneExportSystem exporter;
        
        bool obj_ok = exporter.export_obj(world, "test_export.obj");
        assert(obj_ok);
        std::cout << "  OBJ export: OK" << std::endl;
        
        bool vox_ok = exporter.export_vox(world, "test_export.vox");
        assert(vox_ok);
        std::cout << "  VOX export: OK" << std::endl;
        
        std::cout << "  PASSED" << std::endl;
    }
    
    {
        std::cout << "Test 11: Performance - 100k voxels..." << std::endl;
        VoxelWorld world;
        
        VoxelData v;
        v.albedo = Color(1, 1, 1);
        v.flags = VoxelData::FLAG_SOLID;
        
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int x = 0; x < 50; ++x) {
            for (int y = 0; y < 20; ++y) {
                for (int z = 0; z < 10; ++z) {
                    world.set_voxel(Vec3i64(x, y, z), v);
                }
            }
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        double insert_time = std::chrono::duration<double>(end - start).count();
        
        std::cout << "  Inserted 10,000 voxels in " << insert_time * 1000 << " ms" << std::endl;
        
        start = std::chrono::high_resolution_clock::now();
        
        int count = 0;
        for (int x = 0; x < 50; ++x) {
            for (int y = 0; y < 20; ++y) {
                for (int z = 0; z < 10; ++z) {
                    auto got = world.get_voxel(Vec3i64(x, y, z));
                    if (got) count++;
                }
            }
        }
        
        end = std::chrono::high_resolution_clock::now();
        double query_time = std::chrono::duration<double>(end - start).count();
        
        std::cout << "  Queried " << count << " voxels in " << query_time * 1000 << " ms" << std::endl;
        
        std::cout << "  Memory: " << world.total_memory() / 1024 << " KB" << std::endl;
        std::cout << "  Chunks: " << world.chunks().size() << std::endl;
        
        std::cout << "  PASSED" << std::endl;
    }
    
    {
        std::cout << "Test 12: Dirty chunk tracking..." << std::endl;
        VoxelWorld world;
        
        VoxelData v;
        v.albedo = Color(1, 1, 1);
        v.flags = VoxelData::FLAG_SOLID;
        
        world.set_voxel(Vec3i64(0, 0, 0), v);
        
        auto dirty = world.get_dirty_chunks();
        std::cout << "  Dirty chunks after first edit: " << dirty.size() << std::endl;
        assert(dirty.size() > 0);
        
        world.clear_dirty_chunks();
        dirty = world.get_dirty_chunks();
        assert(dirty.size() == 0);
        
        world.set_voxel(Vec3i64(100, 100, 100), v);
        dirty = world.get_dirty_chunks();
        assert(dirty.size() > 0);
        
        std::cout << "  PASSED" << std::endl;
    }
    
    std::cout << "\n=== All Voxel and GI tests passed! ===" << std::endl;
    return 0;
}
