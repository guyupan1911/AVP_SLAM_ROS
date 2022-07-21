#ifndef PORT_H
#define PORT_H

#include <cinttypes>
#include <cmath>
#include <deque>

namespace AVP
{
    
namespace mapping
{
    
using int8 = int8_t;
using int16 = int16_t;
using int32 = int32_t;
using int64 = int64_t;
using uint8 = uint8_t;
using uint16 = uint16_t;  // 2个字节的无符号整型
using uint32 = uint32_t;
using uint64 = uint64_t;  // 8个字节的无符号整型

namespace common
{
    


inline int RoundToInt(const float x) { return std::lround(x); }

inline int RoundToInt(const double x) { return std::lround(x); }

inline int64 RoundToInt64(const float x) { return std::lround(x); }

inline int64 RoundToInt64(const double x) { return std::lround(x); }

} // namespace common

} // namespace mapping


} // namespace AVP


#endif