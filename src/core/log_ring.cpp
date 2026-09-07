#include "log_ring.hpp"
#include <cstring>

namespace core {

static constexpr size_t kMask = LogRing::kSize - 1;

size_t LogRing::used() const  { return (_head - _tail) & kMask; }
size_t LogRing::space() const { return kSize - 1 - used(); }   // one slot kept free

bool LogRing::push(const void* data, size_t len)
{
    if (len == 0) return true;
    if (len > space()) { _drops++; return false; }

    const uint8_t* p = static_cast<const uint8_t*>(data);
    size_t first = kSize - _head;
    if (first > len) first = len;
    std::memcpy(&_buf[_head], p, first);
    if (len > first) std::memcpy(&_buf[0], p + first, len - first);
    _head = (_head + len) & kMask;
    return true;
}

size_t LogRing::peek(void* out, size_t max) const
{
    size_t n = used();
    if (n > max) n = max;
    uint8_t* o = static_cast<uint8_t*>(out);
    size_t first = kSize - _tail;
    if (first > n) first = n;
    std::memcpy(o, &_buf[_tail], first);
    if (n > first) std::memcpy(o + first, &_buf[0], n - first);
    return n;
}

void LogRing::consume(size_t n)
{
    const size_t u = used();
    if (n > u) n = u;
    _tail = (_tail + n) & kMask;
}

} // namespace core
