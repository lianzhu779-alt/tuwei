#ifndef TMU_MATH_H
#define TMU_MATH_H

#include <math.h>

//
// TMU-friendly math helpers.
// When building with TI Clang (TMU enabled) the __builtin_* versions
// are lowered directly to TMU/FPU instructions. On other toolchains,
// we retain a lightweight fallback so the same code keeps working.
//

#ifndef __has_builtin
#define __has_builtin(x) 0
#endif

#if defined(__TI_COMPILER_VERSION__)
#pragma FUNC_ALWAYS_INLINE(TMU_fabsf)
#endif
static inline float TMU_fabsf(float value)
{
#if __has_builtin(__builtin_fabsf)
    return __builtin_fabsf(value);
#else
    return (value < 0.0f) ? -value : value;
#endif
}

#if defined(__TI_COMPILER_VERSION__)
#pragma FUNC_ALWAYS_INLINE(TMU_fmaxf)
#endif
static inline float TMU_fmaxf(float a, float b)
{
#if __has_builtin(__builtin_fmaxf)
    return __builtin_fmaxf(a, b);
#else
    return (a > b) ? a : b;
#endif
}

#if defined(__TI_COMPILER_VERSION__)
#pragma FUNC_ALWAYS_INLINE(TMU_sqrtf)
#endif
static inline float TMU_sqrtf(float value)
{
#if __has_builtin(__builtin_sqrtf)
    return __builtin_sqrtf(value);
#else
    return sqrtf(value);
#endif
}

#endif // TMU_MATH_H
