#include <iostream>
#include <cassert>
#include <cmath>
#include "voxel_ecs/voxel/voxel.h"
#include "voxel_ecs/water/water.h"
#include "voxel_ecs/network/network.h"
#include "voxel_ecs/material/material.h"
#include "voxel_ecs/terrain/terrain_gen.h"

using namespace voxel_ecs;
using namespace voxel_ecs::voxel;
using namespace voxel_ecs::water;
using namespace voxel_ecs::network;
using namespace voxel_ecs::material;
using namespace voxel_ecs::terrain;

int main() {
    std::cout << "=== New Feature Tests ===" << std::endl;
    
    // Test 1: Water simulation with Gerstner waves
    {
        std::cout << "\nTest 1: Water simulation (Gerstner waves)..." << std::endl;
        
        VoxelWorld world;
        WaterSimulation water(&world, 10);
        
        float height0 = water.get_water_height(0.0f, 0.0f, 0.0f);
        float height1 = water.get_water_height(0.0f, 0.0f, 1.0f);
        
        assert(height0 >= 9.0f && height0 <= 11.0f);
        assert(std::abs(height0 - height1) > 0.0f);
        
        Vec3f normal = water.get_water_normal(0.0f, 0.0f, 0.0f);
        assert(std::abs(length(normal) - 1.0f) < 0.01f);
        assert(normal.y > 0.5f);
        
        Vec3f view(0.0f, -1.0f, 0.0f);
        Vec3f refracted = water.compute_refraction(view, normal);
        assert(length(refracted) > 0.0f);
        
        float caustic = water.compute_caustic_intensity(Vec3f(0.0f, 5.0f, 0.0f), 0.0f);
        assert(caustic >= 0.0f && caustic <= 2.0f);
        
        Color water_color = water.sample_water_color(Vec3f(0, 10, 0), view, Vec3f(1, 1, 1), 0.0f);
        assert(water_color.b > 0.3f);
        
        std::cout << "  PASSED" << std::endl;
    }
    
    // Test 2: Delta encoding for network sync
    {
        std::cout << "\nTest 2: Network Delta encoding..." << std::endl;
        
        VoxelDelta delta;
        delta.position = Vec3i64(100, 50, 200);
        delta.old_data.albedo = Color(0, 0, 0);
        delta.new_data.albedo = Color(1, 0, 0);
        delta.new_data.flags = VoxelData::FLAG_SOLID;
        delta.sequence = 42;
        
        std::vector<uint8_t> encoded = DeltaEncoder::encode(delta);
        assert(encoded.size() > 0 && encoded.size() < 64);
        
        size_t offset = 0;
        VoxelDelta decoded = DeltaEncoder::decode(encoded, offset);
        
        assert(decoded.position == delta.position);
        assert(decoded.new_data.albedo == delta.new_data.albedo);
        assert(decoded.sequence == delta.sequence);
        
        std::vector<VoxelDelta> batch;
        for (int i = 0; i < 5; ++i) {
            VoxelDelta d;
            d.position = Vec3i64(i, i, i);
            d.new_data.albedo = Color(1, 0, 0);
            d.new_data.flags = VoxelData::FLAG_SOLID;
            d.sequence = i;
            batch.push_back(d);
        }
        
        std::vector<uint8_t> encoded_batch = DeltaEncoder::encode_batch(batch);
        size_t batch_offset = 0;
        std::vector<VoxelDelta> decoded_batch = DeltaEncoder::decode_batch(encoded_batch, batch_offset);
        
        assert(decoded_batch.size() == 5);
        for (int i = 0; i < 5; ++i) {
            assert(decoded_batch[i].position == batch[i].position);
        }
        
        std::cout << "  PASSED" << std::endl;
    }
    
    // Test 3: Network sync with mock transport
    {
        std::cout << "\nTest 3: Network sync..." << std::endl;
        
        VoxelWorld server_world;
        VoxelWorld client_world;
        
        auto server_transport = std::make_unique<MockTransport>();
        auto client_transport = std::make_unique<MockTransport>();
        
        MockTransport* server_transport_ptr = server_transport.get();
        MockTransport* client_transport_ptr = client_transport.get();
        
        NetworkSync server_sync(&server_world, std::move(server_transport));
        NetworkSync client_sync(&client_world, std::move(client_transport));
        
        server_sync.start_host(12345);
        client_sync.start_client("localhost", 12345);
        
        assert(server_sync.is_host());
        assert(client_sync.is_connected());
        
        VoxelData v;
        v.albedo = Color(1, 0, 0);
        v.flags = VoxelData::FLAG_SOLID;
        
        for (int i = 0; i < 10; ++i) {
            server_sync.set_voxel(Vec3i64(i, 0, 0), v);
        }
        
        auto outgoing = server_transport_ptr->get_outgoing();
        assert(outgoing.size() == 1);
        
        for (auto& [peer_id, data] : outgoing) {
            client_transport_ptr->inject_packet(0, data);
        }
        
        client_sync.update();
        
        for (int i = 0; i < 10; ++i) {
            auto result = client_world.get_voxel(Vec3i64(i, 0, 0));
            assert(result.has_value());
            assert(result->albedo == Color(1, 0, 0));
        }
        
        std::cout << "  PASSED" << std::endl;
    }
    
    // Test 4: PBR Material system
    {
        std::cout << "\nTest 4: PBR Material system..." << std::endl;
        
        MaterialLibrary library;
        
        PBRMaterial stone;
        stone.albedo = Color(0.6f, 0.6f, 0.6f);
        stone.roughness = 0.9f;
        stone.metallic = 0.0f;
        stone.name = "stone";
        
        PBRMaterial grass;
        grass.albedo = Color(0.2f, 0.5f, 0.1f);
        grass.roughness = 0.8f;
        grass.metallic = 0.0f;
        grass.name = "grass";
        
        uint32_t stone_id = library.add_material(stone);
        uint32_t grass_id = library.add_material(grass);
        
        assert(library.count() == 2);
        assert(library.get_material(stone_id).name == "stone");
        assert(library.get_material(grass_id).name == "grass");
        
        auto* stone_tex = library.get_textures(stone_id);
        auto* grass_tex = library.get_textures(grass_id);
        
        assert(stone_tex != nullptr);
        assert(grass_tex != nullptr);
        
        Color stone_color = stone_tex->albedo.sample(0.5f, 0.5f);
        assert(std::abs(stone_color.r - 0.6f) < 0.01f);
        
        TextureSet<256> transition;
        TransitionTextureGenerator::generate_noise_blend(transition, *stone_tex, *grass_tex, 8.0f, 0.5f, 0.1f);
        
        Color c0 = transition.albedo(0, 0);
        Color c128 = transition.albedo(128, 128);
        Color c255 = transition.albedo(255, 255);
        
        assert(c0 != c255);
        
        Vec3f normal(0, 1, 0);
        Vec3f view(0, -1, 0);
        Vec3f light(1, 1, 1);
        Color lit = PBRRenderer::compute_lighting(grass, normal, view, light, Color(1, 1, 1), 1.0f);
        assert(lit.r >= 0 && lit.g >= 0 && lit.b >= 0);
        
        std::cout << "  PASSED" << std::endl;
    }
    
    // Test 5: L-System for vegetation
    {
        std::cout << "\nTest 5: L-System vegetation generation..." << std::endl;
        
        LSystem lsys;
        lsys.set_axiom("F");
        lsys.add_rule('F', "F[+F]F[-F]F");
        
        std::string gen0 = lsys.generate(0);
        assert(gen0 == "F");
        
        std::string gen1 = lsys.generate(1);
        assert(gen1.find('[') != std::string::npos);
        assert(gen1.find(']') != std::string::npos);
        
        std::string gen2 = lsys.generate(2);
        assert(gen2.length() > gen1.length());
        
        std::cout << "  PASSED" << std::endl;
    }
    
    // Test 6: Tree generator
    {
        std::cout << "\nTest 6: Tree generation..." << std::endl;
        
        VoxelWorld world;
        
        VoxelData ground;
        ground.albedo = Color(0.3f, 0.2f, 0.1f);
        ground.flags = VoxelData::FLAG_SOLID;
        for (int x = -10; x <= 10; ++x) {
            for (int z = -10; z <= 10; ++z) {
                world.set_voxel(Vec3i64(x, 0, z), ground);
            }
        }
        
        TreeGenerator gen(&world);
        
        LSystemParams params;
        params.iterations = 2;
        params.step_length = 1.0f;
        params.angle = 30.0f;
        params.initial_thickness = 1.0f;
        params.seed = 42;
        
        gen.generate_tree(Vec3i64(0, 1, 0), params, 0);
        
        int trunk_count = 0;
        int leaf_count = 0;
        
        for (int y = 1; y < 20; ++y) {
            for (int x = -5; x <= 5; ++x) {
                for (int z = -5; z <= 5; ++z) {
                    auto v = world.get_voxel(Vec3i64(x, y, z));
                    if (v) {
                        if (v->albedo == Color(0.35f, 0.2f, 0.1f)) {
                            trunk_count++;
                        } else if (v->is_vegetation() && v->albedo == Color(0.15f, 0.45f, 0.1f)) {
                            leaf_count++;
                        }
                    }
                }
            }
        }
        
        assert(trunk_count > 0);
        assert(leaf_count > 0);
        
        std::cout << "  Trunk voxels: " << trunk_count << ", Leaf voxels: " << leaf_count << std::endl;
        std::cout << "  PASSED" << std::endl;
    }
    
    // Test 7: Vine generation
    {
        std::cout << "\nTest 7: Vine generation..." << std::endl;
        
        VoxelWorld world;
        TreeGenerator gen(&world);
        
        gen.generate_vine(Vec3i64(0, 5, 0), Vec3f(1.0f, 0.2f, 0.0f), 20, 42);
        
        int vine_count = 0;
        for (int x = 0; x < 20; ++x) {
            for (int y = 0; y < 10; ++y) {
                for (int z = -5; z <= 5; ++z) {
                    auto v = world.get_voxel(Vec3i64(x, y, z));
                    if (v && v->albedo == Color(0.2f, 0.5f, 0.15f)) {
                        vine_count++;
                    }
                }
            }
        }
        
        assert(vine_count > 5);
        std::cout << "  Vine voxels: " << vine_count << std::endl;
        std::cout << "  PASSED" << std::endl;
    }
    
    // Test 8: Jungle population
    {
        std::cout << "\nTest 8: Jungle population..." << std::endl;
        
        VoxelWorld world;
        
        VoxelData ground;
        ground.albedo = Color(0.3f, 0.2f, 0.1f);
        ground.flags = VoxelData::FLAG_SOLID;
        for (int x = 0; x < 30; ++x) {
            for (int z = 0; z < 30; ++z) {
                world.set_voxel(Vec3i64(x, 0, z), ground);
            }
        }
        
        TreeGenerator gen(&world);
        AABB bounds(Vec3f(0, 0, 0), Vec3f(30, 20, 30));
        gen.populate_jungle(bounds, 0.5f, 42);
        
        int total_vegetation = 0;
        for (int x = 0; x < 30; ++x) {
            for (int y = 1; y < 20; ++y) {
                for (int z = 0; z < 30; ++z) {
                    auto v = world.get_voxel(Vec3i64(x, y, z));
                    if (v && v->is_vegetation()) {
                        total_vegetation++;
                    }
                }
            }
        }
        
        assert(total_vegetation > 50);
        std::cout << "  Total vegetation voxels: " << total_vegetation << std::endl;
        std::cout << "  PASSED" << std::endl;
    }
    
    // Test 9: Water surface update
    {
        std::cout << "\nTest 9: Water surface update..." << std::endl;
        
        VoxelWorld world;
        WaterSimulation water(&world, 5);
        
        AABB bounds(Vec3f(-5, 0, -5), Vec3f(5, 10, 5));
        water.update_water_surface(bounds, 0.0f);
        
        int water_count = 0;
        for (int x = -5; x <= 5; ++x) {
            for (int y = 3; y <= 7; ++y) {
                for (int z = -5; z <= 5; ++z) {
                    auto v = world.get_voxel(Vec3i64(x, y, z));
                    if (v && v->is_water()) {
                        water_count++;
                    }
                }
            }
        }
        
        assert(water_count > 100);
        std::cout << "  Water voxels: " << water_count << std::endl;
        std::cout << "  PASSED" << std::endl;
    }
    
    std::cout << "\n=== All new feature tests passed! ===" << std::endl;
    
    return 0;
}
