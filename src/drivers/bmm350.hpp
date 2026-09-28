#pragma once
namespace mag350 {
struct Sample { float x_ut,y_ut,z_ut,temp_c; };
bool begin();
bool poll(Sample& out);
bool healthy();
}
