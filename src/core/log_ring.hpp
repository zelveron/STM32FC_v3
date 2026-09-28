#pragma once
//
// log_ring.hpp -- byte ring buffer between the logging producers and the SD
// flush task.
//
// Single producer, single consumer, both on the one cooperative thread, so no
// locking is needed. push() is O(len) memcpy and never blocks: if the record
// does not fit it is dropped whole and counted. The storage task copies up to
// 492 stream bytes into a protected 512-byte sector envelope.
//
// Portable.
//
#include <cstddef>
#include <cstdint>

namespace core {

class LogRing {
public:
    // About 0.8 s at the default dual-IMU + control + state log rates.
    // Longer stalls drop new complete records; control never waits for room.
    static constexpr size_t kSize = 32768;

    // Append len bytes atomically. Returns false and bumps drops() if the ring
    // cannot hold the whole record right now.
    bool push(const void* data, size_t len);

    // Copy up to max of the oldest bytes into out (handles wraparound).
    // Does not consume -- call consume() once the bytes are safely on media.
    size_t peek(void* out, size_t max) const;

    // Drop the n oldest bytes.
    void consume(size_t n);

    size_t   used()  const;
    size_t   space() const;
    uint32_t drops() const { return _drops; }

private:
    uint8_t  _buf[kSize];
    size_t   _head  = 0;   // next write
    size_t   _tail  = 0;   // next read
    uint32_t _drops = 0;
};

} // namespace core
