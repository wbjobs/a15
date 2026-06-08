#include <voxel_ecs/voxel_ecs.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>

using namespace voxel_ecs;
using namespace voxel_ecs::math;

void save_ppm(const std::string& filename, const std::vector<uint8_t>& framebuffer, 
              uint32_t width, uint32_t height) {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    
    file << "P6\n" << width << " " << height << "\n255\n";
    
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            uint32_t idx = (y * width + x) * 4;
            file.put(framebuffer[idx]);
            file.put(framebuffer[idx + 1]);
            file.put(framebuffer[idx + 2]);
        }
    }
    
    std::cout << "Saved: " << filename << std::endl;
}

int main() {
    std::cout << "=== Voxel ECS Engine with DDGI ===" << std::endl;
    
    const uint32_t WIDTH = 640;
    const uint32_t HEIGHT = 480;
    
    VoxelEngine engine(WIDTH, HEIGHT);
    
    std::cout << "Generating test scene..." << std::endl;
    engine.generate_test_scene();
    
    engine.create_camera(Vec3f(0, 8, 15), Vec3f(0, -0.4f, 0));
    
    std::cout << "Initializing DDGI probe grid..." << std::endl;
    engine.initialize_gi(8, 4.0f);
    
    std::cout << "Voxel world memory: " << engine.world().total_memory() / 1024 << " KB" << std::endl;
    std::cout << "Number of chunks: " << engine.world().chunks().size() << std::endl;
    
    std::cout << "\nBaking initial GI (this may take a moment)..." << std::endl;
    auto bake_start = std::chrono::high_resolution_clock::now();
    engine.bake_gi(128);
    auto bake_end = std::chrono::high_resolution_clock::now();
    double bake_time = std::chrono::duration<double>(bake_end - bake_start).count();
    std::cout << "GI bake time: " << bake_time << "s" << std::endl;
    
    std::cout << "\nRendering frame 1 (initial view)..." << std::endl;
    auto render_start = std::chrono::high_resolution_clock::now();
    engine.update(0.016f);
    engine.render();
    auto render_end = std::chrono::high_resolution_clock::now();
    double render_time = std::chrono::duration<double>(render_end - render_start).count();
    std::cout << "Render time: " << render_time * 1000 << " ms" << std::endl;
    
    save_ppm("frame1_initial.ppm", engine.renderer().framebuffer(), WIDTH, HEIGHT);
    
    std::cout << "\n=== Testing voxel editing ===" << std::endl;
    std::cout << "Adding a tower of voxels..." << std::endl;
    for (int y = 1; y < 8; ++y) {
        voxel::VoxelData brick;
        brick.albedo = Color(0.8f, 0.2f, 0.2f);
        brick.flags = voxel::VoxelData::FLAG_SOLID;
        engine.edit_voxel(Vec3i64(8, y, 0), voxel::VoxelEditEvent::Type::ADD, brick);
    }
    
    std::cout << "Removing some voxels from the tower..." << std::endl;
    engine.edit_voxel(Vec3i64(8, 3, 0), voxel::VoxelEditEvent::Type::REMOVE);
    engine.edit_voxel(Vec3i64(8, 5, 0), voxel::VoxelEditEvent::Type::REMOVE);
    
    engine.update(0.016f);
    
    std::cout << "Rendering frame 2 (after edits, before GI update)..." << std::endl;
    engine.render();
    save_ppm("frame2_after_edit.ppm", engine.renderer().framebuffer(), WIDTH, HEIGHT);
    
    std::cout << "\nGI probes updating incrementally..." << std::endl;
    for (int i = 0; i < 5; ++i) {
        engine.update(0.016f);
    }
    
    std::cout << "Rendering frame 3 (after GI update)..." << std::endl;
    engine.render();
    save_ppm("frame3_gi_updated.ppm", engine.renderer().framebuffer(), WIDTH, HEIGHT);
    
    std::cout << "\n=== Testing camera movement ===" << std::endl;
    engine.move_camera_forward(-2.0f);
    engine.move_camera_right(3.0f);
    engine.rotate_camera(0.3f, 0.1f);
    
    std::cout << "Rendering frame 4 (different angle)..." << std::endl;
    engine.render();
    save_ppm("frame4_different_angle.ppm", engine.renderer().framebuffer(), WIDTH, HEIGHT);
    
    std::cout << "\n=== Testing scene export ===" << std::endl;
    if (engine.export_scene_obj("scene.obj")) {
        std::cout << "Exported scene.obj" << std::endl;
    } else {
        std::cerr << "Failed to export OBJ" << std::endl;
    }
    
    if (engine.export_scene_vox("scene.vox")) {
        std::cout << "Exported scene.vox" << std::endl;
    } else {
        std::cerr << "Failed to export VOX" << std::endl;
    }
    
    std::cout << "\n=== Testing ECS direct access ===" << std::endl;
    auto& ecs_world = engine.ecs_world();
    
    ecs::EntityId test_entity = ecs_world.create_entity();
    
    struct Position { float x, y, z; };
    struct Velocity { float vx, vy, vz; };
    
    ecs_world.entities().add_component<Position>(test_entity, 0.0f, 0.0f, 0.0f);
    ecs_world.entities().add_component<Velocity>(test_entity, 1.0f, 2.0f, 3.0f);
    
    std::cout << "Created entity: " << test_entity << std::endl;
    std::cout << "Has Position: " << ecs_world.entities().has_component<Position>(test_entity) << std::endl;
    std::cout << "Has Velocity: " << ecs_world.entities().has_component<Velocity>(test_entity) << std::endl;
    
    auto view = ecs::make_view<Position, Velocity>(
        ecs_world.entities(),
        [](ecs::EntityId id, Position& pos, Velocity& vel) {
            pos.x += vel.vx * 0.016f;
            pos.y += vel.vy * 0.016f;
            pos.z += vel.vz * 0.016f;
            std::cout << "Entity " << id << " position: (" 
                      << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
        }
    );
    view.each();
    
    std::cout << "\n=== Running performance test ===" << std::endl;
    const int NUM_FRAMES = 10;
    double total_time = 0;
    
    for (int i = 0; i < NUM_FRAMES; ++i) {
        engine.move_camera_right(0.1f);
        
        auto start = std::chrono::high_resolution_clock::now();
        engine.update(0.016f);
        engine.render();
        auto end = std::chrono::high_resolution_clock::now();
        
        double frame_time = std::chrono::duration<double>(end - start).count();
        total_time += frame_time;
        
        if (i % 5 == 0) {
            std::cout << "Frame " << i << ": " << frame_time * 1000 << " ms, " 
                      << 1.0 / frame_time << " FPS" << std::endl;
        }
    }
    
    std::cout << "\nAverage frame time: " << (total_time / NUM_FRAMES) * 1000 << " ms" << std::endl;
    std::cout << "Average FPS: " << NUM_FRAMES / total_time << std::endl;
    
    std::cout << "\n=== Testing SVDAG optimization ===" << std::endl;
    size_t mem_before = engine.world().total_memory();
    engine.world().optimize_all();
    size_t mem_after = engine.world().total_memory();
    std::cout << "Memory before optimization: " << mem_before / 1024 << " KB" << std::endl;
    std::cout << "Memory after optimization: " << mem_after / 1024 << " KB" << std::endl;
    std::cout << "Savings: " << (mem_before - mem_after) / 1024 << " KB (" 
              << (100.0 * (mem_before - mem_after) / mem_before) << "%)" << std::endl;
    
    std::cout << "\n=== All tests completed ===" << std::endl;
    std::cout << "Output files:" << std::endl;
    std::cout << "  - frame1_initial.ppm" << std::endl;
    std::cout << "  - frame2_after_edit.ppm" << std::endl;
    std::cout << "  - frame3_gi_updated.ppm" << std::endl;
    std::cout << "  - frame4_different_angle.ppm" << std::endl;
    std::cout << "  - scene.obj" << std::endl;
    std::cout << "  - scene.vox" << std::endl;
    
    return 0;
}
