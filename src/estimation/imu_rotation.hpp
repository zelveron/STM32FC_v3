#pragma once
#include <cstdint>
#include <cstdlib>
namespace estimation {
inline bool rotation_valid(const int8_t (&axis)[3]) {
    int m[3][3]{};
    for(unsigned i=0;i<3;++i) {
        const int j=std::abs(int(axis[i]))-1;
        if(j<0 || j>2) return false;
        m[i][j]=axis[i]<0?-1:1;
    }
    return m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])-
           m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])+
           m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0])==1;
}
inline void rotate(const int8_t (&axis)[3],const float (&sensor)[3],float (&body)[3]) {
    for(unsigned i=0;i<3;++i) body[i]=(axis[i]<0?-1.0f:1.0f)*sensor[std::abs(int(axis[i]))-1];
}
}
