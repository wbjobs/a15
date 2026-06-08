#pragma once

#include "../core/math.h"
#include "../voxel/voxel.h"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <cstring>
#include <memory>
#include <functional>
#include <chrono>

namespace voxel_ecs::network {

using namespace math;
using namespace voxel;

enum class PacketType : uint8_t {
    VOXEL_DELTA = 1,
    VOXEL_BATCH = 2,
    CHUNK_REQUEST = 3,
    CHUNK_DATA = 4,
    WORLD_STATE = 5,
    ACK = 6,
    PING = 7,
    PONG = 8
};

struct VoxelDelta {
    Vec3i64 position;
    VoxelData old_data;
    VoxelData new_data;
    uint64_t timestamp;
    uint32_t author_id;
    uint32_t sequence;
};

struct PacketHeader {
    PacketType type;
    uint32_t size;
    uint32_t sequence;
    uint32_t sender_id;
    uint64_t timestamp;
};

class DeltaEncoder {
public:
    static std::vector<uint8_t> encode(const VoxelDelta& delta) {
        std::vector<uint8_t> buffer;
        buffer.reserve(64);
        
        uint64_t pos_x = static_cast<uint64_t>(delta.position.x);
        uint64_t pos_y = static_cast<uint64_t>(delta.position.y);
        uint64_t pos_z = static_cast<uint64_t>(delta.position.z);
        
        encode_zigzag(buffer, pos_x);
        encode_zigzag(buffer, pos_y);
        encode_zigzag(buffer, pos_z);
        
        uint8_t flags = 0;
        if (delta.old_data.albedo != delta.new_data.albedo) flags |= 1 << 0;
        if (delta.old_data.roughness != delta.new_data.roughness) flags |= 1 << 1;
        if (delta.old_data.metallic != delta.new_data.metallic) flags |= 1 << 2;
        if (delta.old_data.emission != delta.new_data.emission) flags |= 1 << 3;
        if (delta.old_data.flags != delta.new_data.flags) flags |= 1 << 4;
        
        buffer.push_back(flags);
        
        if (flags & (1 << 0)) {
            encode_color(buffer, delta.new_data.albedo);
        }
        if (flags & (1 << 1)) {
            encode_float(buffer, delta.new_data.roughness);
        }
        if (flags & (1 << 2)) {
            encode_float(buffer, delta.new_data.metallic);
        }
        if (flags & (1 << 3)) {
            encode_float(buffer, delta.new_data.emission);
        }
        if (flags & (1 << 4)) {
            buffer.push_back(delta.new_data.flags);
            buffer.push_back(delta.new_data.material_id);
        }
        
        encode_uint32(buffer, delta.sequence);
        
        return buffer;
    }
    
    static VoxelDelta decode(const std::vector<uint8_t>& buffer, size_t& offset) {
        VoxelDelta delta;
        
        uint64_t pos_x = decode_zigzag(buffer, offset);
        uint64_t pos_y = decode_zigzag(buffer, offset);
        uint64_t pos_z = decode_zigzag(buffer, offset);
        
        delta.position = Vec3i64(
            static_cast<int64_t>(pos_x),
            static_cast<int64_t>(pos_y),
            static_cast<int64_t>(pos_z)
        );
        
        uint8_t flags = buffer[offset++];
        
        if (flags & (1 << 0)) {
            delta.new_data.albedo = decode_color(buffer, offset);
        }
        if (flags & (1 << 1)) {
            delta.new_data.roughness = decode_float(buffer, offset);
        }
        if (flags & (1 << 2)) {
            delta.new_data.metallic = decode_float(buffer, offset);
        }
        if (flags & (1 << 3)) {
            delta.new_data.emission = decode_float(buffer, offset);
        }
        if (flags & (1 << 4)) {
            delta.new_data.flags = buffer[offset++];
            delta.new_data.material_id = buffer[offset++];
        }
        
        delta.sequence = decode_uint32(buffer, offset);
        
        return delta;
    }
    
    static std::vector<uint8_t> encode_batch(const std::vector<VoxelDelta>& deltas) {
        std::vector<uint8_t> buffer;
        buffer.reserve(deltas.size() * 32 + 8);
        
        encode_uint32(buffer, static_cast<uint32_t>(deltas.size()));
        
        for (const auto& delta : deltas) {
            auto encoded = encode(delta);
            buffer.insert(buffer.end(), encoded.begin(), encoded.end());
        }
        
        return buffer;
    }
    
    static std::vector<VoxelDelta> decode_batch(const std::vector<uint8_t>& buffer, size_t& offset) {
        uint32_t count = decode_uint32(buffer, offset);
        std::vector<VoxelDelta> deltas;
        deltas.reserve(count);
        
        for (uint32_t i = 0; i < count; ++i) {
            deltas.push_back(decode(buffer, offset));
        }
        
        return deltas;
    }
    
private:
    static void encode_zigzag(std::vector<uint8_t>& buffer, uint64_t value) {
        uint64_t encoded = (value << 1) ^ (static_cast<int64_t>(value) >> 63);
        while (encoded >= 0x80) {
            buffer.push_back(static_cast<uint8_t>(encoded | 0x80));
            encoded >>= 7;
        }
        buffer.push_back(static_cast<uint8_t>(encoded));
    }
    
    static uint64_t decode_zigzag(const std::vector<uint8_t>& buffer, size_t& offset) {
        uint64_t result = 0;
        int shift = 0;
        while (offset < buffer.size()) {
            uint8_t b = buffer[offset++];
            result |= static_cast<uint64_t>(b & 0x7F) << shift;
            if (!(b & 0x80)) break;
            shift += 7;
        }
        return (result >> 1) ^ -(result & 1);
    }
    
    static void encode_float(std::vector<uint8_t>& buffer, float value) {
        uint32_t u;
        std::memcpy(&u, &value, sizeof(u));
        buffer.push_back(u & 0xFF);
        buffer.push_back((u >> 8) & 0xFF);
        buffer.push_back((u >> 16) & 0xFF);
        buffer.push_back((u >> 24) & 0xFF);
    }
    
    static float decode_float(const std::vector<uint8_t>& buffer, size_t& offset) {
        uint32_t u = static_cast<uint32_t>(buffer[offset]) |
                    (static_cast<uint32_t>(buffer[offset + 1]) << 8) |
                    (static_cast<uint32_t>(buffer[offset + 2]) << 16) |
                    (static_cast<uint32_t>(buffer[offset + 3]) << 24);
        offset += 4;
        float f;
        std::memcpy(&f, &u, sizeof(f));
        return f;
    }
    
    static void encode_color(std::vector<uint8_t>& buffer, const Color& color) {
        buffer.push_back(static_cast<uint8_t>(std::clamp(color.r, 0.0f, 1.0f) * 255.0f));
        buffer.push_back(static_cast<uint8_t>(std::clamp(color.g, 0.0f, 1.0f) * 255.0f));
        buffer.push_back(static_cast<uint8_t>(std::clamp(color.b, 0.0f, 1.0f) * 255.0f));
    }
    
    static Color decode_color(const std::vector<uint8_t>& buffer, size_t& offset) {
        uint8_t r = buffer[offset++];
        uint8_t g = buffer[offset++];
        uint8_t b = buffer[offset++];
        Color c(
            static_cast<float>(r) / 255.0f,
            static_cast<float>(g) / 255.0f,
            static_cast<float>(b) / 255.0f
        );
        return c;
    }
    
    static void encode_uint32(std::vector<uint8_t>& buffer, uint32_t value) {
        buffer.push_back(value & 0xFF);
        buffer.push_back((value >> 8) & 0xFF);
        buffer.push_back((value >> 16) & 0xFF);
        buffer.push_back((value >> 24) & 0xFF);
    }
    
    static uint32_t decode_uint32(const std::vector<uint8_t>& buffer, size_t& offset) {
        uint32_t value = static_cast<uint32_t>(buffer[offset]) |
                        (static_cast<uint32_t>(buffer[offset + 1]) << 8) |
                        (static_cast<uint32_t>(buffer[offset + 2]) << 16) |
                        (static_cast<uint32_t>(buffer[offset + 3]) << 24);
        offset += 4;
        return value;
    }
};

class INetworkTransport {
public:
    virtual ~INetworkTransport() = default;
    virtual bool connect(const std::string& address, uint16_t port) = 0;
    virtual bool host(uint16_t port, uint32_t max_clients = 16) = 0;
    virtual void disconnect() = 0;
    virtual void send(uint32_t peer_id, const std::vector<uint8_t>& data, bool reliable = true) = 0;
    virtual void broadcast(const std::vector<uint8_t>& data, bool reliable = true) = 0;
    virtual std::vector<std::pair<uint32_t, std::vector<uint8_t>>> receive() = 0;
    virtual bool is_connected() const = 0;
    virtual bool is_host() const = 0;
    virtual uint32_t client_id() const = 0;
};

class MockTransport : public INetworkTransport {
public:
    MockTransport() = default;
    
    bool connect(const std::string& address, uint16_t port) override {
        connected_ = true;
        is_host_ = false;
        client_id_ = 1;
        return true;
    }
    
    bool host(uint16_t port, uint32_t max_clients = 16) override {
        connected_ = true;
        is_host_ = true;
        client_id_ = 0;
        max_clients_ = max_clients;
        return true;
    }
    
    void disconnect() override {
        connected_ = false;
        outgoing_.clear();
        incoming_.clear();
    }
    
    void send(uint32_t peer_id, const std::vector<uint8_t>& data, bool reliable = true) override {
        if (connected_) {
            outgoing_.push_back({peer_id, data});
        }
    }
    
    void broadcast(const std::vector<uint8_t>& data, bool reliable = true) override {
        if (connected_ && is_host_) {
            for (uint32_t i = 1; i <= max_clients_; ++i) {
                outgoing_.push_back({i, data});
            }
        }
    }
    
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> receive() override {
        auto result = std::move(incoming_);
        incoming_.clear();
        return result;
    }
    
    bool is_connected() const override { return connected_; }
    bool is_host() const override { return is_host_; }
    uint32_t client_id() const override { return client_id_; }
    
    void inject_packet(uint32_t from_peer, const std::vector<uint8_t>& data) {
        incoming_.push_back({from_peer, data});
    }
    
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> get_outgoing() {
        auto result = std::move(outgoing_);
        outgoing_.clear();
        return result;
    }
    
private:
    bool connected_{false};
    bool is_host_{false};
    uint32_t client_id_{0};
    uint32_t max_clients_{16};
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> outgoing_;
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> incoming_;
};

class NetworkSync {
public:
    NetworkSync(VoxelWorld* world, std::unique_ptr<INetworkTransport> transport)
        : world_(world), transport_(std::move(transport)) {}
    
    void start_host(uint16_t port, uint32_t max_clients = 16) {
        if (transport_->host(port, max_clients)) {
            is_host_ = true;
        }
    }
    
    void start_client(const std::string& address, uint16_t port) {
        if (transport_->connect(address, port)) {
            is_host_ = false;
        }
    }
    
    void stop() {
        transport_->disconnect();
    }
    
    void set_voxel(const Vec3i64& pos, const VoxelData& new_data) {
        auto old_data = world_->get_voxel(pos);
        
        VoxelDelta delta;
        delta.position = pos;
        delta.old_data = old_data.value_or(VoxelData{});
        delta.new_data = new_data;
        delta.timestamp = get_timestamp();
        delta.author_id = transport_->client_id();
        delta.sequence = next_sequence_++;
        
        pending_deltas_.push_back(delta);
        
        world_->set_voxel(pos, new_data);
        
        if (pending_deltas_.size() >= batch_size_) {
            flush_batch();
        }
    }
    
    void remove_voxel(const Vec3i64& pos) {
        auto old_data = world_->get_voxel(pos);
        if (!old_data) return;
        
        VoxelDelta delta;
        delta.position = pos;
        delta.old_data = *old_data;
        delta.new_data = VoxelData{};
        delta.timestamp = get_timestamp();
        delta.author_id = transport_->client_id();
        delta.sequence = next_sequence_++;
        
        pending_deltas_.push_back(delta);
        
        world_->remove_voxel(pos);
        
        if (pending_deltas_.size() >= batch_size_) {
            flush_batch();
        }
    }
    
    void flush_batch() {
        if (pending_deltas_.empty()) return;
        
        std::vector<uint8_t> packet;
        
        PacketHeader header;
        header.type = PacketType::VOXEL_BATCH;
        header.sequence = next_packet_sequence_++;
        header.sender_id = transport_->client_id();
        header.timestamp = get_timestamp();
        
        auto payload = DeltaEncoder::encode_batch(pending_deltas_);
        header.size = static_cast<uint32_t>(payload.size());
        
        packet.reserve(sizeof(PacketHeader) + payload.size());
        packet.insert(packet.end(), 
            reinterpret_cast<uint8_t*>(&header), 
            reinterpret_cast<uint8_t*>(&header) + sizeof(PacketHeader));
        packet.insert(packet.end(), payload.begin(), payload.end());
        
        if (is_host_) {
            transport_->broadcast(packet);
        } else {
            transport_->send(0, packet);
        }
        
        history_.insert(history_.end(), pending_deltas_.begin(), pending_deltas_.end());
        if (history_.size() > max_history_) {
            history_.erase(history_.begin(), history_.begin() + (history_.size() - max_history_));
        }
        
        pending_deltas_.clear();
    }
    
    void update() {
        auto messages = transport_->receive();
        
        for (auto& [sender_id, data] : messages) {
            if (data.size() < sizeof(PacketHeader)) continue;
            
            PacketHeader* header = reinterpret_cast<PacketHeader*>(data.data());
            
            switch (header->type) {
                case PacketType::VOXEL_BATCH: {
                    if (header->sender_id == transport_->client_id()) break;
                    
                    size_t offset = sizeof(PacketHeader);
                    std::vector<uint8_t> payload(data.begin() + sizeof(PacketHeader), data.end());
                    
                    size_t payload_offset = 0;
                    auto deltas = DeltaEncoder::decode_batch(payload, payload_offset);
                    
                    for (const auto& delta : deltas) {
                        apply_delta(delta);
                    }
                    
                    send_ack(header->sender_id, header->sequence);
                    break;
                }
                
                case PacketType::ACK: {
                    size_t offset = sizeof(PacketHeader);
                    uint32_t ack_seq = *reinterpret_cast<uint32_t*>(data.data() + offset);
                    acknowledged_sequences_.insert(ack_seq);
                    break;
                }
                
                default:
                    break;
            }
        }
        
        if (!pending_deltas_.empty()) {
            flush_batch();
        }
    }
    
    void set_batch_size(uint32_t size) { batch_size_ = size; }
    
    bool is_host() const { return is_host_; }
    bool is_connected() const { return transport_->is_connected(); }
    
    const std::vector<VoxelDelta>& history() const { return history_; }
    
private:
    void apply_delta(const VoxelDelta& delta) {
        if (delta.new_data.flags == 0 && delta.new_data.albedo.r == 0 && 
            delta.new_data.albedo.g == 0 && delta.new_data.albedo.b == 0) {
            world_->remove_voxel(delta.position);
        } else {
            world_->set_voxel(delta.position, delta.new_data);
        }
        
        applied_deltas_.push_back(delta);
        if (applied_deltas_.size() > max_history_) {
            applied_deltas_.erase(applied_deltas_.begin());
        }
    }
    
    void send_ack(uint32_t target_id, uint32_t sequence) {
        std::vector<uint8_t> packet;
        
        PacketHeader header;
        header.type = PacketType::ACK;
        header.size = sizeof(uint32_t);
        header.sequence = next_packet_sequence_++;
        header.sender_id = transport_->client_id();
        header.timestamp = get_timestamp();
        
        packet.insert(packet.end(), 
            reinterpret_cast<uint8_t*>(&header), 
            reinterpret_cast<uint8_t*>(&header) + sizeof(PacketHeader));
        
        uint32_t seq = sequence;
        packet.insert(packet.end(), 
            reinterpret_cast<uint8_t*>(&seq), 
            reinterpret_cast<uint8_t*>(&seq) + sizeof(uint32_t));
        
        transport_->send(target_id, packet);
    }
    
    uint64_t get_timestamp() const {
        return static_cast<uint64_t>(
            std::chrono::high_resolution_clock::now().time_since_epoch().count() / 1000
        );
    }
    
    VoxelWorld* world_;
    std::unique_ptr<INetworkTransport> transport_;
    bool is_host_{false};
    
    uint32_t batch_size_{16};
    uint32_t next_sequence_{0};
    uint32_t next_packet_sequence_{0};
    
    std::vector<VoxelDelta> pending_deltas_;
    std::vector<VoxelDelta> history_;
    std::vector<VoxelDelta> applied_deltas_;
    std::unordered_set<uint32_t> acknowledged_sequences_;
    
    static constexpr uint32_t max_history_ = 1024;
};

struct NetworkComponent {
    NetworkSync* sync;
    float update_interval{0.05f};
    float time_since_update{0.0f};
};

class NetworkSystem : public ecs::System<NetworkSystem> {
public:
    static constexpr const char* system_name() { return "NetworkSystem"; }
    
    void update_impl(ecs::EntityManager& em, float dt) {
        auto view = ecs::make_view<NetworkComponent>(
            em, [dt](ecs::EntityId, NetworkComponent& net) {
                net.time_since_update += dt;
                if (net.time_since_update >= net.update_interval) {
                    if (net.sync) {
                        net.sync->update();
                    }
                    net.time_since_update = 0.0f;
                }
            }
        );
        view.each();
    }
};

}
