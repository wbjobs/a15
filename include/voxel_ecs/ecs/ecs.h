#pragma once

#include <cstdint>
#include <vector>
#include <tuple>
#include <type_traits>
#include <bitset>
#include <array>
#include <unordered_map>
#include <memory>
#include <algorithm>
#include <numeric>
#include <cassert>

namespace voxel_ecs::ecs {

using EntityId = uint64_t;
using ComponentId = uint32_t;
using SystemId = uint32_t;

constexpr EntityId INVALID_ENTITY = ~0ULL;
constexpr ComponentId MAX_COMPONENTS = 64;
constexpr uint32_t ECS_CHUNK_SIZE = 256;

using ComponentMask = std::bitset<MAX_COMPONENTS>;

namespace detail {
    inline ComponentId get_next_component_id() {
        static ComponentId next_id = 0;
        return next_id++;
    }
    
    inline SystemId get_next_system_id() {
        static SystemId next_id = 0;
        return next_id++;
    }
}

template <typename T>
struct ComponentType {
    static ComponentId id;
    static constexpr const char* name = typeid(T).name();
};

template <typename T>
ComponentId ComponentType<T>::id = detail::get_next_component_id();

template <typename... Ts>
struct ComponentList {
    static constexpr size_t count = sizeof...(Ts);
    
    template <typename T>
    static constexpr bool contains() {
        return (std::is_same_v<T, Ts> || ...);
    }
    
    static ComponentMask mask() {
        ComponentMask m;
        ((m.set(ComponentType<std::remove_cvref_t<Ts>>::id)), ...);
        return m;
    }
};

template <typename T>
class ComponentStorage {
public:
    ComponentStorage() {
        ensure_capacity(ECS_CHUNK_SIZE);
    }
    
    T& get(EntityId entity) {
        assert(entity_to_index_.count(entity) && "Entity does not have this component");
        return data_[entity_to_index_[entity]];
    }
    
    const T& get(EntityId entity) const {
        assert(entity_to_index_.count(entity) && "Entity does not have this component");
        return data_[entity_to_index_.at(entity)];
    }
    
    bool has(EntityId entity) const {
        return entity_to_index_.count(entity) > 0;
    }
    
    template <typename... Args>
    T& add(EntityId entity, Args&&... args) {
        assert(!entity_to_index_.count(entity) && "Entity already has this component");
        
        if (data_.size() >= capacity_) {
            ensure_capacity(capacity_ * 2);
        }
        
        uint32_t index = static_cast<uint32_t>(data_.size());
        data_.emplace_back(std::forward<Args>(args)...);
        entity_to_index_[entity] = index;
        index_to_entity_[index] = entity;
        return data_.back();
    }
    
    void remove(EntityId entity) {
        assert(entity_to_index_.count(entity) && "Entity does not have this component");
        
        uint32_t remove_index = entity_to_index_[entity];
        uint32_t last_index = static_cast<uint32_t>(data_.size() - 1);
        
        if (remove_index != last_index) {
            std::swap(data_[remove_index], data_[last_index]);
            EntityId last_entity = index_to_entity_[last_index];
            entity_to_index_[last_entity] = remove_index;
            index_to_entity_[remove_index] = last_entity;
        }
        
        data_.pop_back();
        entity_to_index_.erase(entity);
        index_to_entity_.erase(last_index);
    }
    
    const std::vector<T>& data() const { return data_; }
    
    auto begin() { return data_.begin(); }
    auto end() { return data_.end(); }
    auto begin() const { return data_.begin(); }
    auto end() const { return data_.end(); }
    
    EntityId entity_at(uint32_t index) const {
        return index_to_entity_.at(index);
    }
    
    size_t size() const { return data_.size(); }
    
private:
    void ensure_capacity(size_t new_capacity) {
        data_.reserve(new_capacity);
        capacity_ = new_capacity;
    }
    
    std::vector<T> data_;
    std::unordered_map<EntityId, uint32_t> entity_to_index_;
    std::unordered_map<uint32_t, EntityId> index_to_entity_;
    size_t capacity_ = 0;
};

template <typename T>
using ComponentStoragePtr = std::unique_ptr<ComponentStorage<T>>;

class IComponentStorage {
public:
    virtual ~IComponentStorage() = default;
    virtual bool has(EntityId entity) const = 0;
    virtual void remove(EntityId entity) = 0;
    virtual size_t size() const = 0;
};

template <typename T>
class ComponentStorageWrapper : public IComponentStorage {
public:
    ComponentStorageWrapper(ComponentStoragePtr<T> storage) : storage_(std::move(storage)) {}
    
    bool has(EntityId entity) const override { return storage_->has(entity); }
    void remove(EntityId entity) override { storage_->remove(entity); }
    size_t size() const override { return storage_->size(); }
    
    ComponentStorage<T>& get() { return *storage_; }
    const ComponentStorage<T>& get() const { return *storage_; }
    
private:
    ComponentStoragePtr<T> storage_;
};

class EntityManager {
public:
    EntityId create() {
        EntityId id;
        if (!free_list_.empty()) {
            id = free_list_.back();
            free_list_.pop_back();
        } else {
            id = next_id_++;
        }
        entity_masks_[id].reset();
        return id;
    }
    
    void destroy(EntityId entity) {
        for (auto& [comp_id, storage] : components_) {
            if (storage->has(entity)) {
                storage->remove(entity);
            }
        }
        entity_masks_.erase(entity);
        free_list_.push_back(entity);
    }
    
    bool alive(EntityId entity) const {
        return entity_masks_.count(entity) > 0;
    }
    
    const ComponentMask& mask(EntityId entity) const {
        return entity_masks_.at(entity);
    }
    
    template <typename T>
    ComponentStorage<T>& storage() {
        ComponentId id = ComponentType<T>::id;
        if (components_.find(id) == components_.end()) {
            components_[id] = std::make_unique<ComponentStorageWrapper<T>>(
                std::make_unique<ComponentStorage<T>>()
            );
        }
        return static_cast<ComponentStorageWrapper<T>*>(components_[id].get())->get();
    }
    
    template <typename T>
    const ComponentStorage<T>& storage() const {
        ComponentId id = ComponentType<T>::id;
        return static_cast<const ComponentStorageWrapper<T>*>(components_.at(id).get())->get();
    }
    
    template <typename T, typename... Args>
    T& add_component(EntityId entity, Args&&... args) {
        T& comp = storage<T>().add(entity, std::forward<Args>(args)...);
        entity_masks_[entity].set(ComponentType<T>::id);
        return comp;
    }
    
    template <typename T>
    void remove_component(EntityId entity) {
        storage<T>().remove(entity);
        entity_masks_[entity].reset(ComponentType<T>::id);
    }
    
    template <typename T>
    T& get_component(EntityId entity) {
        return storage<T>().get(entity);
    }
    
    template <typename T>
    const T& get_component(EntityId entity) const {
        return storage<T>().get(entity);
    }
    
    template <typename T>
    bool has_component(EntityId entity) const {
        ComponentId id = ComponentType<T>::id;
        auto it = components_.find(id);
        if (it == components_.end()) return false;
        return it->second->has(entity);
    }
    
    const std::unordered_map<EntityId, ComponentMask>& entity_masks() const {
        return entity_masks_;
    }
    
private:
    EntityId next_id_ = 0;
    std::vector<EntityId> free_list_;
    std::unordered_map<EntityId, ComponentMask> entity_masks_;
    std::unordered_map<ComponentId, std::unique_ptr<IComponentStorage>> components_;
};

template <typename Func, typename... Components>
class View {
public:
    View(EntityManager& em, Func func) 
        : em_(em), func_(std::move(func)), 
          required_mask_(ComponentList<Components...>::mask()) {}
    
    void each() {
        for (auto& [entity, mask] : em_.entity_masks()) {
            if ((mask & required_mask_) == required_mask_) {
                func_(entity, em_.get_component<Components>(entity)...);
            }
        }
    }
    
    template <typename Predicate>
    void each_if(Predicate pred) {
        for (auto& [entity, mask] : em_.entity_masks()) {
            if ((mask & required_mask_) == required_mask_ && pred(entity)) {
                func_(entity, em_.get_component<Components>(entity)...);
            }
        }
    }
    
    void parallel_each() {
        const auto& entities = em_.entity_masks();
        std::vector<EntityId> matching;
        matching.reserve(entities.size());
        
        for (auto& [entity, mask] : entities) {
            if ((mask & required_mask_) == required_mask_) {
                matching.push_back(entity);
            }
        }
        
        #pragma omp parallel for
        for (int64_t i = 0; i < static_cast<int64_t>(matching.size()); ++i) {
            EntityId e = matching[i];
            func_(e, em_.get_component<Components>(e)...);
        }
    }
    
private:
    EntityManager& em_;
    Func func_;
    ComponentMask required_mask_;
};

template <typename... Components, typename Func>
auto make_view(EntityManager& em, Func func) {
    return View<Func, Components...>(em, std::move(func));
}

class ISystem {
public:
    virtual ~ISystem() = default;
    virtual void update(EntityManager& em, float dt) = 0;
    virtual SystemId id() const = 0;
    virtual const char* name() const = 0;
};

template <typename Derived>
class System : public ISystem {
public:
    static SystemId system_id() {
        static SystemId id = detail::get_next_system_id();
        return id;
    }
    
    SystemId id() const override { return system_id(); }
    const char* name() const override { return Derived::system_name(); }
    
    virtual void update(EntityManager& em, float dt) override {
        static_cast<Derived*>(this)->update_impl(em, dt);
    }
};

class SystemManager {
public:
    template <typename T, typename... Args>
    T& add_system(Args&&... args) {
        SystemId id = T::system_id();
        auto system = std::make_unique<T>(std::forward<Args>(args)...);
        T* ptr = system.get();
        systems_[id] = std::move(system);
        order_.push_back(id);
        return *ptr;
    }
    
    template <typename T>
    T& get_system() {
        return *static_cast<T*>(systems_[T::system_id()].get());
    }
    
    void update(EntityManager& em, float dt) {
        for (SystemId id : order_) {
            systems_[id]->update(em, dt);
        }
    }
    
    void update_single(EntityManager& em, SystemId id, float dt) {
        systems_[id]->update(em, dt);
    }
    
private:
    std::unordered_map<SystemId, std::unique_ptr<ISystem>> systems_;
    std::vector<SystemId> order_;
};

class World {
public:
    EntityManager& entities() { return entities_; }
    SystemManager& systems() { return systems_; }
    
    EntityId create_entity() {
        return entities_.create();
    }
    
    void destroy_entity(EntityId entity) {
        entities_.destroy(entity);
    }
    
    void update(float dt) {
        systems_.update(entities_, dt);
    }
    
private:
    EntityManager entities_;
    SystemManager systems_;
};

}
