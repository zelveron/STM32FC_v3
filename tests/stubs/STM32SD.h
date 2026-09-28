#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>
constexpr int FR_OK=0,FILE_WRITE=1;
namespace fake_sd {
inline bool fail_write=false,fail_flush=false,full=false;
inline int error=0;
inline unsigned exists_calls=0;
inline std::vector<uint8_t> data;
}
class File {
public:
    explicit operator bool() const { return true; }
    size_t write(const uint8_t* b,size_t n) {
        if(fake_sd::fail_write) { fake_sd::error=1; return 0; }
        fake_sd::data.insert(fake_sd::data.end(),b,b+n); return n;
    }
    void flush() { if(fake_sd::fail_flush) fake_sd::error=1; }
    int getErrorstate() const { return fake_sd::error; }
};
struct FakeSD {
    bool begin() { fake_sd::error=0; fake_sd::exists_calls=0; return true; }
    bool exists(const char*) { ++fake_sd::exists_calls; return fake_sd::full; }
    File open(const char*,int) { return {}; }
};
inline FakeSD SD;
