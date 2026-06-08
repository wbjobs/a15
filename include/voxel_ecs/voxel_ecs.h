#pragma once

#include "core/math.h"
#include "ecs/ecs.h"
#include "voxel/voxel.h"
#include "gi/ddgi.h"
#include "export/scene_export.h"
#include "render/software_renderer.h"

namespace voxel_ecs {

using namespace math;
using namespace ecs;

class VoxelEngine {
public:
    VoxelEngine(uint32_t render_width = 800, uint32_t render_height = 600) {
        world_ = std::make_unique<voxel::VoxelWorld>();
        
        ecs::EntityId world_entity = ecs_world_.create_entity();
        ecs_world_.entities().add_component<voxel::VoxelWorldComponent>(
            world_entity, world_.get()
        );
        
        ecs_world_.systems().add_system<voxel::VoxelEditSystem>();
        ecs_world_.systems().add_system<gi::DDGISystem>();
        ecs_world_.systems().add_system<export_::SceneExportSystem>();
        ecs_world_.systems().add_system<render::CameraSystem>();
        ecs_world_.systems().add_system<render::RenderSystem>(render_width, render_height);
    }
    
    void initialize_gi(int64_t grid_size = 16, float spacing = 4.0f) {
        gi_grid_ = std::make_unique<gi::DDGIProbeGrid>(
            Vec3i64(0, 0, 0),
            Vec3i64(grid_size, grid_size, grid_size),
            spacing
        );
        
        gi_updater_ = std::make_unique<gi::ProbeUpdateSystem>(world_.get(), gi_grid_.get());
        
        ecs::EntityId gi_entity = ecs_world_.create_entity();
        ecs_world_.entities().add_component<gi::DDGIComponent>(
            gi_entity, gi_grid_.get(), gi_updater_.get()
        );
    }
    
    void create_camera(const Vec3f& position = Vec3f(0, 5, 10), 
                       const Vec3f& rotation = Vec3f(0, -0.3f, 0)) {
        ecs::EntityId cam_entity = ecs_world_.create_entity();
        ecs_world_.entities().add_component<render::CameraComponent>(
            cam_entity, position, rotation
        );
    }
    
    void generate_test_scene() {
        for (int x = -20; x < 20; ++x) {
            for (int z = -20; z < 20; ++z) {
                voxel::VoxelData grass;
                grass.albedo = Color(0.3f, 0.7f, 0.2f);
                grass.flags = voxel::VoxelData::FLAG_SOLID;
                world_->set_voxel(Vec3i64(x, 0, z), grass);
                
                if (std::abs(x) < 15 && std::abs(z) < 15) {
                    voxel::VoxelData dirt;
                    dirt.albedo = Color(0.5f, 0.35f, 0.2f);
                    dirt.flags = voxel::VoxelData::FLAG_SOLID;
                    world_->set_voxel(Vec3i64(x, -1, z), dirt);
                }
            }
        }
        
        for (int y = 1; y < 6; ++y) {
            for (int x = -3; x <= 3; ++x) {
                for (int z = -3; z <= 3; ++z) {
                    if (y == 5 && std::abs(x) < 3 && std::abs(z) < 3) continue;
                    if (y < 5 && std::abs(x) == 3 && std::abs(z) == 3) continue;
                    
                    voxel::VoxelData stone;
                    stone.albedo = Color(0.6f, 0.6f, 0.65f);
                    stone.flags = voxel::VoxelData::FLAG_SOLID;
                    world_->set_voxel(Vec3i64(x, y, z), stone);
                }
            }
        }
        
        for (int i = 0; i < 3; ++i) {
            int bx = -10 + i * 8;
            int bz = -10 + i * 6;
            
            for (int y = 1; y < 4; ++y) {
                for (int x = bx - 2; x <= bx + 2; ++x) {
                    for (int z = bz - 2; z <= bz + 2; ++z) {
                        if (y < 3 || (std::abs(x - bx) <= 1 && std::abs(z - bz) <= 1)) {
                            voxel::VoxelData wood;
                            wood.albedo = Color(0.45f, 0.3f, 0.15f);
                            wood.flags = voxel::VoxelData::FLAG_SOLID;
                            world_->set_voxel(Vec3i64(x, y, z), wood);
                        }
                    }
                }
            }
            
            for (int y = 4; y < 8; ++y) {
                int r = 7 - y;
                for (int x = bx - r; x <= bx + r; ++x) {
                    for (int z = bz - r; z <= bz + r; ++z) {
                        int dx = x - bx;
                        int dz = z - bz;
                        if (dx * dx + dz * dz <= r * r) {
                            voxel::VoxelData leaves;
                            leaves.albedo = Color(0.15f, 0.5f, 0.1f);
                            leaves.flags = voxel::VoxelData::FLAG_SOLID;
                            world_->set_voxel(Vec3i64(x, y, z), leaves);
                        }
                    }
                }
            }
        }
        
        voxel::VoxelData light;
        light.albedo = Color(1.0f, 0.9f, 0.6f);
        light.emission = 2.0f;
        light.flags = voxel::VoxelData::FLAG_SOLID;
        world_->set_voxel(Vec3i64(5, 8, 5), light);
        world_->set_voxel(Vec3i64(-8, 6, -8), light);
    }
    
    void edit_voxel(const Vec3i64& pos, voxel::VoxelEditEvent::Type type, 
                    const voxel::VoxelData& data = voxel::VoxelData{}) {
        auto& edit_system = ecs_world_.systems().get_system<voxel::VoxelEditSystem>();
        edit_system.add_edit({type, pos, data});
    }
    
    void update(float dt) {
        ecs_world_.update(dt);
    }
    
    void render() {
        ecs_world_.systems().update_single(
            ecs_world_.entities(), 
            render::RenderSystem::system_id(), 
            0.0f
        );
    }
    
    void bake_gi(uint32_t num_rays = 256) {
        if (gi_updater_) {
            gi_updater_->update_all_probes(num_rays);
        }
    }
    
    bool export_scene_obj(const std::string& filepath) {
        auto& export_system = ecs_world_.systems().get_system<export_::SceneExportSystem>();
        return export_system.export_obj(*world_, filepath);
    }
    
    bool export_scene_vox(const std::string& filepath) {
        auto& export_system = ecs_world_.systems().get_system<export_::SceneExportSystem>();
        return export_system.export_vox(*world_, filepath);
    }
    
    void move_camera_forward(float speed) {
        auto view = ecs::make_view<render::CameraComponent>(
            ecs_world_.entities(),
            [this, speed](ecs::EntityId, render::CameraComponent& cam) {
                auto& cam_sys = ecs_world_.systems().get_system<render::CameraSystem>();
                cam_sys.move_forward(cam, speed);
            }
        );
        view.each();
    }
    
    void move_camera_right(float speed) {
        auto view = ecs::make_view<render::CameraComponent>(
            ecs_world_.entities(),
            [this, speed](ecs::EntityId, render::CameraComponent& cam) {
                auto& cam_sys = ecs_world_.systems().get_system<render::CameraSystem>();
                cam_sys.move_right(cam, speed);
            }
        );
        view.each();
    }
    
    void move_camera_up(float speed) {
        auto view = ecs::make_view<render::CameraComponent>(
            ecs_world_.entities(),
            [this, speed](ecs::EntityId, render::CameraComponent& cam) {
                auto& cam_sys = ecs_world_.systems().get_system<render::CameraSystem>();
                cam_sys.move_up(cam, speed);
            }
        );
        view.each();
    }
    
    void rotate_camera(float yaw, float pitch) {
        auto view = ecs::make_view<render::CameraComponent>(
            ecs_world_.entities(),
            [this, yaw, pitch](ecs::EntityId, render::CameraComponent& cam) {
                auto& cam_sys = ecs_world_.systems().get_system<render::CameraSystem>();
                cam_sys.rotate(cam, yaw, pitch);
            }
        );
        view.each();
    }
    
    const render::SoftwareRenderer& renderer() const {
        return const_cast<ecs::World&>(ecs_world_).systems().get_system<render::RenderSystem>().renderer();
    }
    
    voxel::VoxelWorld& world() { return *world_; }
    ecs::World& ecs_world() { return ecs_world_; }
    
private:
    ecs::World ecs_world_;
    std::unique_ptr<voxel::VoxelWorld> world_;
    std::unique_ptr<gi::DDGIProbeGrid> gi_grid_;
    std::unique_ptr<gi::ProbeUpdateSystem> gi_updater_;
};

}
