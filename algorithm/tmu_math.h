#ifndef TMU_MATH_H
#define TMU_MATH_H

#include <math.h>

#if defined(__TMS320C28XX_CLA__)
#include "CLAmath.h"
#endif

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


#if defined(__TI_COMPILER_VERSION__)
#pragma FUNC_ALWAYS_INLINE(TMU_sincosf)
#endif

#if defined(__TI_COMPILER_VERSION__)
#pragma FUNC_ALWAYS_INLINE(TMU_rsqrtf)
#endif
static inline float TMU_rsqrtf(float value)
{
#if __has_builtin(__builtin_rsqrtf)
    return __builtin_rsqrtf(value);
#else
    return 1.0f / TMU_sqrtf(value);
#endif
}


static inline void TMU_sincosf(float angle, float *sine, float *cosine)
{
#if defined(__TMS320C28XX_CLA__)
    CLAsincos_inline(angle, sine, cosine);
#elif __has_builtin(__builtin_sinf) && __has_builtin(__builtin_cosf)
    *sine = __builtin_sinf(angle);
    *cosine = __builtin_cosf(angle);
#else
    *sine = sinf(angle);
    *cosine = cosf(angle);
#endif
}

#endif // TMU_MATH_H
