#include "usb_stream.hpp"

// BENCH SCAFFOLDING -- see usb_stream.hpp. Arduino-only, excluded from native.

namespace {

class NbLog : public Print {
public:
    size_t write(uint8_t c) override
    {
        if (c == '\n') { flush_line(); return 1; }
        if (_len < sizeof(_buf)) _buf[_len++] = (char)c;
        else                     _overflow = true;
        return 1;
    }
    size_t write(const uint8_t* b, size_t n) override
    {
        for (size_t i = 0; i < n; i++) write(b[i]);
        return n;
    }
    uint32_t drops() const { return _drops; }

private:
    void flush_line()
    {
        if (!_overflow && SerialUSB.availableForWrite() >= (int)(_len + 1)) {
            SerialUSB.write((const uint8_t*)_buf, _len);
            SerialUSB.write((uint8_t)'\n');
        } else {
            _drops++;
        }
        _len = 0;
        _overflow = false;
    }

    char     _buf[288];
    uint16_t _len = 0;
    bool     _overflow = false;
    uint32_t _drops = 0;
};

NbLog s_log;

} // namespace

namespace usb_stream {

void      begin()      { SerialUSB.begin(); }
bool      host_ready() { return (bool)SerialUSB; }
Print&    log()        { return s_log; }
uint32_t  drops()      { return s_log.drops(); }
int       read()       { return SerialUSB.available() ? SerialUSB.read() : -1; }

} // namespace usb_stream
