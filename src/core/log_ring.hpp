#pragma once
//
// log_ring.hpp -- byte ring buffer between the logging producers and the SD
// flush task.
//
// Single producer, single consumer, both on the one cooperative thread, so no
// locking is needed. push() is O(len) memcpy and never blocks: if the record
// does not fit it is dropped whole and counted. The flush task pulls
// 512-byte-aligned chunks.
//
// Portable.
//
#include <cstddef>
#include <cstdint>

namespace core {

class LogRing {
public:
    // ~3.5 s of log at the current frame rate. Must stay a power of two. Sized
    // so a multi-hundred-ms SD stall (worst-case card hiccup) never drops a
    // frame, while leaving RAM headroom for later work (F407 has 128 KB).
    static constexpr size_t kSize = 16384;

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
