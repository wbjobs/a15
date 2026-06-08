#include <voxel_ecs/ecs/ecs.h>
#include <iostream>
#include <cassert>
#include <chrono>

using namespace voxel_ecs;
using namespace voxel_ecs::ecs;

struct Position {
    float x, y, z;
};

struct Velocity {
    float vx, vy, vz;
};

struct Health {
    float hp;
    float max_hp;
};

class MovementSystem : public System<MovementSystem> {
public:
    static constexpr const char* system_name() { return "MovementSystem"; }
    
    void update_impl(EntityManager& em, float dt) {
        auto view = make_view<Position, Velocity>(
            em, [dt](EntityId, Position& pos, Velocity& vel) {
                pos.x += vel.vx * dt;
                pos.y += vel.vy * dt;
                pos.z += vel.vz * dt;
            }
        );
        view.each();
    }
};

class HealthSystem : public System<HealthSystem> {
public:
    static constexpr const char* system_name() { return "HealthSystem"; }
    
    void update_impl(EntityManager& em, float dt) {
        auto view = make_view<Health>(
            em, [](EntityId, Health& h) {
                if (h.hp < h.max_hp) {
                    h.hp = std::min(h.hp + 0.1f, h.max_hp);
                }
            }
        );
        view.each();
    }
};

int main() {
    std::cout << "=== ECS Tests ===" << std::endl;
    
    {
        std::cout << "Test 1: Entity creation and destruction..." << std::endl;
        EntityManager em;
        
        EntityId e1 = em.create();
        EntityId e2 = em.create();
        EntityId e3 = em.create();
        
        assert(em.alive(e1));
        assert(em.alive(e2));
        assert(em.alive(e3));
        assert(!em.alive(1000));
        
        em.destroy(e2);
        assert(!em.alive(e2));
        
        EntityId e4 = em.create();
        assert(e4 == e2);
        
        std::cout << "  PASSED" << std::endl;
    }
    
    {
        std::cout << "Test 2: Component addition and retrieval..." << std::endl;
        EntityManager em;
        EntityId e = em.create();
        
        em.add_component<Position>(e, 1.0f, 2.0f, 3.0f);
        em.add_component<Velocity>(e, 0.5f, 0.0f, -0.5f);
        
        assert(em.has_component<Position>(e));
        assert(em.has_component<Velocity>(e));
        assert(!em.has_component<Health>(e));
        
        Position& pos = em.get_component<Position>(e);
        assert(pos.x == 1.0f);
        assert(pos.y == 2.0f);
        assert(pos.z == 3.0f);
        
        std::cout << "  PASSED" << std::endl;
    }
    
    {
        std::cout << "Test 3: Component removal..." << std::endl;
        EntityManager em;
        EntityId e = em.create();
        
        em.add_component<Position>(e, 1.0f, 2.0f, 3.0f);
        em.add_component<Velocity>(e, 0.5f, 0.0f, -0.5f);
        
        em.remove_component<Velocity>(e);
        assert(!em.has_component<Velocity>(e));
        assert(em.has_component<Position>(e));
        
        std::cout << "  PASSED" << std::endl;
    }
    
    {
        std::cout << "Test 4: Component mask..." << std::endl;
        EntityManager em;
        EntityId e = em.create();
        
        em.add_component<Position>(e, 1.0f, 2.0f, 3.0f);
        em.add_component<Velocity>(e, 0.5f, 0.0f, -0.5f);
        
        const ComponentMask& mask = em.mask(e);
        assert(mask.test(ComponentType<Position>::id));
        assert(mask.test(ComponentType<Velocity>::id));
        assert(!mask.test(ComponentType<Health>::id));
        
        std::cout << "  PASSED" << std::endl;
    }
    
    {
        std::cout << "Test 5: View iteration..." << std::endl;
        EntityManager em;
        
        for (int i = 0; i < 100; ++i) {
            EntityId e = em.create();
            em.add_component<Position>(e, i * 1.0f, 0, 0);
            if (i % 2 == 0) {
                em.add_component<Velocity>(e, 1.0f, 0, 0);
            }
        }
        
        int count = 0;
        auto view = make_view<Position, Velocity>(
            em, [&count](EntityId, Position& p, Velocity& v) {
                count++;
                p.x += v.vx * 0.1f;
            }
        );
        view.each();
        
        assert(count == 50);
        
        for (int i = 0; i < 100; i += 2) {
            EntityId e = i;
            Position& p = em.get_component<Position>(e);
            assert(p.x == i * 1.0f + 0.1f);
        }
        
        std::cout << "  PASSED" << std::endl;
    }
    
    {
        std::cout << "Test 6: System manager..." << std::endl;
        World world;
        
        auto& move_sys = world.systems().add_system<MovementSystem>();
        auto& health_sys = world.systems().add_system<HealthSystem>();
        
        EntityId e = world.create_entity();
        world.entities().add_component<Position>(e, 0.0f, 0.0f, 0.0f);
        world.entities().add_component<Velocity>(e, 1.0f, 2.0f, 3.0f);
        world.entities().add_component<Health>(e, 50.0f, 100.0f);
        
        world.update(1.0f);
        
        Position& p = world.entities().get_component<Position>(e);
        assert(p.x == 1.0f);
        assert(p.y == 2.0f);
        assert(p.z == 3.0f);
        
        Health& h = world.entities().get_component<Health>(e);
        assert(h.hp == 50.1f);
        
        std::cout << "  PASSED" << std::endl;
    }
    
    {
        std::cout << "Test 7: Component storage performance..." << std::endl;
        EntityManager em;
        
        const int NUM_ENTITIES = 1000;
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < NUM_ENTITIES; ++i) {
            EntityId e = em.create();
            em.add_component<Position>(e, i, i, i);
            if (i % 2 == 0) {
                em.add_component<Velocity>(e, 1, 1, 1);
            }
            if (i % 3 == 0) {
                em.add_component<Health>(e, 100, 100);
            }
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        double time = std::chrono::duration<double>(end - start).count();
        std::cout << "  Created " << NUM_ENTITIES << " entities in " << time * 1000 << " ms" << std::endl;
        
        start = std::chrono::high_resolution_clock::now();
        int iterations = 0;
        auto view = make_view<Position, Velocity>(
            em, [&iterations](EntityId, Position& p, Velocity& v) {
                p.x += v.vx * 0.016f;
                iterations++;
            }
        );
        view.each();
        end = std::chrono::high_resolution_clock::now();
        time = std::chrono::duration<double>(end - start).count();
        
        std::cout << "  Iterated " << iterations << " entities in " << time * 1000 << " ms" << std::endl;
        assert(iterations == NUM_ENTITIES / 2);
        std::cout << "  PASSED" << std::endl;
    }
    
    {
        std::cout << "Test 8: Entity destruction with components..." << std::endl;
        EntityManager em;
        
        for (int i = 0; i < 10; ++i) {
            EntityId e = em.create();
            em.add_component<Position>(e, i, i, i);
            em.add_component<Velocity>(e, 1, 1, 1);
        }
        
        assert(em.storage<Position>().size() == 10);
        assert(em.storage<Velocity>().size() == 10);
        
        em.destroy(5);
        
        assert(em.storage<Position>().size() == 9);
        assert(em.storage<Velocity>().size() == 9);
        assert(!em.alive(5));
        
        std::cout << "  PASSED" << std::endl;
    }
    
    {
        std::cout << "Test 9: Predicate view..." << std::endl;
        EntityManager em;
        
        for (int i = 0; i < 10; ++i) {
            EntityId e = em.create();
            em.add_component<Health>(e, i * 10.0f, 100.0f);
        }
        
        int count = 0;
        auto view = make_view<Health>(
            em, [&count](EntityId, Health& h) {
                count++;
                h.hp = 0;
            }
        );
        view.each_if([&em](EntityId e) {
            return em.get_component<Health>(e).hp < 50.0f;
        });
        
        assert(count == 5);
        
        for (int i = 0; i < 5; ++i) {
            assert(em.get_component<Health>(i).hp == 0);
        }
        for (int i = 5; i < 10; ++i) {
            assert(em.get_component<Health>(i).hp == i * 10.0f);
        }
        
        std::cout << "  PASSED" << std::endl;
    }
    
    std::cout << "\n=== All ECS tests passed! ===" << std::endl;
    return 0;
}
