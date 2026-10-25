#include <string.h>
#if DEBUG
#include <assert.h>
#endif
#include "trig.h"
#include "game/math.h"

static const u16 gCubicBSplineBasis[2] = {
    Q_4_12(2.0 / 3.0), Q_4_12(1.0 / 6.0)
};

static u32 gRngPrevValue = 0;
static u32 gRngValue = 0;

static inline bool8 TransformNode_RemoveChild(TransformNode *parent, TransformNode *child);

inline u16 Random(void)
{
    u32 *pPrev = &gRngPrevValue;
#if !PORTABLE
    u16 *pCurrent = ((u16 *)&gRngValue);
#endif

    u32 prev = *pPrev;
    *pPrev = gRngValue;

    gRngValue = prev + RAND_CONST;
    gRngValue += *pPrev;

#if PORTABLE
    // For the sake of big-endian platforms...
    return gRngValue >> 16;
#else
    return pCurrent[1];
#endif
}

inline void SeedRng(u32 prev, u32 cur)
{
    gRngPrevValue = prev;
    gRngValue = cur;
}

UNUSED inline u32 MaxAbs(s32 a, s32 b)
{
    a = ABS(a);
    b = ABS(b);

    return MAX(a, b);
}

inline u32 ApproxVecLen(s32 a, s32 b)
{
    a = ABS(a);
    b = ABS(b);

    return (a <= b)
          ? I((u64)b * Q(123.0 / 128.0) + (u64)a * Q(51.0 / 128.0)) :
            I((u64)a * Q(123.0 / 128.0) + (u64)b * Q(51.0 / 128.0));
}

inline s16 AngleDiff(u16 angleA, u16 angleB, u8 n)
{
    u32 range = BitValue(n);
    u16 mask = range - 1;

    angleB = (angleB - angleA) & mask;

    return (angleB <= range / 2) ? angleB : angleB - range;
}

inline u32 Lerp(s32 start, s32 end, s32 t, u8 n) { return start - ((s64)t * (end - start) >> n); }

UNUSED inline u16 LerpAngle(u16 angleStart, u16 angleEnd, u8 n1, s16 t, u8 n2)
{
    s16 diff = -(t * -AngleDiff(angleStart, angleEnd, n1) >> n2);

    return (angleStart + diff) & (BitValue(n1) - 1);
}

inline s32 LerpToEnd(s32 start, s32 end, s32 t, u8 n, u8 cnt)
{
    do {
        start = Lerp(start, end, t, n);
    } while (--cnt != 255);

    return start;
}

inline s32 LerpToStart(s32 start, s32 end, s32 t, u8 n, u8 cnt)
{
    do {
        end = Lerp(start, end, t, n);
    } while (--cnt != 255);

    return end;
}

UNUSED inline s32 WeightedLeft(s32 prev, s32 cur, s32 next) { return (prev * 7 + cur * 6 - next) / 12; }

UNUSED inline s32 WeightedRight(s32 prev, s32 cur, s32 next) { return (-prev + cur * 6 + next * 7) / 12; }

UNUSED inline s32 WeightedLeft2(s32 prev, s32 cur, s32 next) { return (prev + cur * 8 - next) >> 3; }

UNUSED inline s32 WeightedRight2(s32 prev, s32 cur, s32 next) { return (-prev + cur * 8 + next) >> 3; }

inline s32 DeltaScaled(s32 start, s32 end, s32 t, UNUSED s32 adjust, UNUSED s32 offset, u8 n) { return (s64)(end - start) * (BitValue(n) - t) >> n; }

inline s32 Delta(s32 start, s32 end) { return end - start; }

inline s32 CurveOffsetScaled(s32 prev, s32 cur, s32 next, s32 t, UNUSED s32 adjust, UNUSED s32 offset, u8 n)
{
    prev = next - prev;
    prev += prev >> 1;
    prev -= (cur >> 1);

    return (s64)prev * (BitValue(n) - t) >> n;
}

inline s32 CurveOffset(s32 prev, s32 cur, s32 next)
{
    s32 delta = Delta(prev, next);

    return delta + (delta >> 1) - (cur >> 1);
}

UNUSED inline s32 CurveOffsetScaled2(s32 prev, s32 cur, s32 next, s32 t, s32 adjust, s32 offset, u8 n) { return CurveOffsetScaled(prev, cur, next, t, adjust, offset, n); }

UNUSED inline s32 CurveOffset2(s32 prev, s32 cur, s32 next) { return CurveOffset(prev, cur, next); }

UNUSED inline s32 DeltaCurveOffsetScaled(s32 prev, s32 cur, s32 next, s32 t, s32 adjust, s32 offset, u8 n)
{
    s32 prevDelta = DeltaScaled(prev, cur, -offset, adjust, offset, n);
    s32 nextDelta = DeltaScaled(cur, next, offset, adjust, offset, n);
#ifndef NON_MATCHING
    s64 tmp = prevDelta;
#endif
    s64 interpolatedDelta = ((s64)Delta(prevDelta, nextDelta) * ((BitValue(n - 1)) + (adjust >> 1))) >> n;

    return ((s64)(interpolatedDelta + prevDelta) * (BitValue(n) - t)) >> n;
}

UNUSED inline s32 DeltaCurveOffsetScaledInverse(s32 prev, s32 cur, s32 next, s32 t, s32 adjust, s32 offset, u8 n)
{
    s32 prevDelta = DeltaScaled(prev, cur, -offset, adjust, offset, n);
    s32 nextDelta = DeltaScaled(cur, next, offset, adjust, offset, n);
#ifndef NON_MATCHING
    s64 tmp = prevDelta;
#endif
    s64 interpolatedDelta = ((s64)Delta(prevDelta, nextDelta) * ((BitValue(n - 1)) - (adjust >> 1))) >> n;

    return ((s64)(interpolatedDelta + prevDelta) * (BitValue(n) - t)) >> n;
}

UNUSED inline s32 DeltaCentral(s32 prev, s32 cur, s32 next)
{
    s32 delta = Delta(prev, cur);

    return delta + ((Delta(cur, next) - delta) >> 1);
}

inline s16 CubicBSplineSegment(s16 *b, u16 t)
{
    s32 result, t2, t3;

    t3 = (Q_4_12(1.0) - 1) - t;
    t3 = Q_4_12_TO_INT(t3 * Q_4_12_TO_INT(SQUARE(t3)));

    result = Q_6_10_TO_INT(t3 * *b++ * (Q_6_10(1.0 / 6.0) + 1));

    t2 = Q_4_12_TO_INT(SQUARE(t));
    t3 = Q_4_12_TO_INT(t2 * t);

    result += *b++ * ((t3 >> 1) - t2 + gCubicBSplineBasis[0]);
    result += *b++ * (((t + t2 - t3) >> 1) + gCubicBSplineBasis[1]);
    result += Q_6_10_TO_INT(t3 * *b * (Q_6_10(1.0 / 6.0) + 1));

    return Q_4_12_TO_INT(result);
}

inline u16 CubicBSpline(s16 *b, u16 t)
{
    u32 result;
    s32 t2, t3;

    b = &b[t / Q_4_12(1.0)];
    t %= Q_4_12(1.0);

    t3 = (Q_4_12(1.0) - 1) - t;
    t3 = Q_4_12_TO_INT(t3 * Q_4_12_TO_INT(SQUARE(t3)));

    result = Q_6_10_TO_INT(t3 * b[0] * (Q_6_10(1.0 / 6.0) + 1));

    t2 = Q_4_12_TO_INT(SQUARE(t));
    t3 = Q_4_12_TO_INT(t2 * t);

    result += b[1] * ((t3 >> 1) - t2 + gCubicBSplineBasis[0]);
    result += (b += 2)[0] * (((t + t2 - t3) >> 1) + gCubicBSplineBasis[1]);
    result += Q_6_10_TO_INT(t3 * b[1] * (Q_6_10(1.0 / 6.0) + 1));

    return Q_4_12_TO_INT(Q_6_10_TO_INT(Q_6_10(result)));
}

#if (GAME == GAME_SA3)
inline s16 CubicBSplineDirect(s16 *b, u16 t)
{
    s32 result, t2, t3;

    b = &b[t / Q_4_12(1.0)];
    t %= Q_4_12(1.0);

    t3 = (Q_4_12(1.0) - 1) - t;
    t3 = Q_4_12_TO_INT(t3 * Q_4_12_TO_INT(SQUARE(t3)));

    result = Q_6_10_TO_INT(t3 * b[0] * (Q_6_10(1.0 / 6.0) + 1));

    t2 = Q_4_12_TO_INT(SQUARE(t));
    t3 = Q_4_12_TO_INT(t2 * t);

    result += b[1] * ((t3 >> 1) - t2 + gCubicBSplineBasis[0]);
    result += (b += 2)[0] * (((t + t2 - t3) >> 1) + gCubicBSplineBasis[1]);
    result += Q_6_10_TO_INT(t3 * b[1] * (Q_6_10(1.0 / 6.0) + 1));

    return Q_4_12_TO_INT(result);
}
#endif

UNUSED s32 BezierCubicInterpolate(s32 p0, s32 p1, s32 p2, s32 p3, s32 t, u8 n)
{
    s32 t2 = SQUARE((s64)t) >> n;
    s32 t3 = (s64)t2 * t >> n;

    s64 invT = BitValue(n) - t;
    s32 invT2 = SQUARE(invT) >> n;
    s32 invT3 = invT2 * invT >> n;

    return ((s64)invT3 * p0 +
            ((s64)invT2 * t >> n) * p1 * 3 +
            (invT * t2 >> n) * p2 * 3 +
            (s64)t3 * p3) >> n;
}

UNUSED s32 HermiteCubicInterpolate(s32 p0, s32 m0, s32 m1, s32 p1, s32 t, u8 n)
{
    s32 t2 = SQUARE((s64)t) >> n;
    s32 t3 = (s64)t2 * t >> n;

    return ((s64)p0 * ((t3 - t2) * 2 - t2 + BitValue(n)) +
            (s64)m0 * (-t2 + t3 - t2 + t) +
            (s64)p1 * (t2 * 3 - t3 - t3) +
            (s64)m1 * (-t2 + t3)) >> n;
}

UNUSED inline void Transform2D_Init(Transform2D *transform)
{
    transform->rotation = 0;
    transform->scaleX = Q_6_10(1.0);
    transform->scaleY = Q_6_10(1.0);
    transform->translation.x = 0;
    transform->translation.y = 0;
}

UNUSED void Transform2D_Apply(Transform2D *local, Transform2D *parent, Transform2D *out)
{
    s32 x, y;

    if (parent->scaleX != Q_6_10(1.0)) {
        x = Q_6_10_MUL(parent->scaleX, local->translation.x);
        out->scaleX = Q_6_10_MUL(parent->scaleX, local->scaleX);
    } else {
        x = local->translation.x;
    }

    if (parent->scaleY != Q_6_10(1.0)) {
        y = Q_6_10_MUL(parent->scaleY, local->translation.y);
        out->scaleY = Q_6_10_MUL(parent->scaleY, local->scaleY);
    } else {
        y = local->translation.y;
    }

    if (parent->rotation) {
        out->translation.x = parent->translation.x + (Q_MUL(x, COS_24_8(parent->rotation)) - Q_MUL(y, SIN_24_8(parent->rotation)));
        out->translation.y = parent->translation.y + (Q_MUL(x, SIN_24_8(parent->rotation)) + Q_MUL(y, COS_24_8(parent->rotation)));
    } else {
        out->translation.x = parent->translation.x + x;
        out->translation.y = parent->translation.y + y;
    }

    out->rotation = CLAMP_SIN_PERIOD(local->rotation + parent->rotation);
}

static void Transform2D_ApplyToLocal(Transform2D *local, Transform2D *parent)
{
    s32 x, y;

    if (parent->scaleX != Q_6_10(1.0)) {
        x = Q_6_10_MUL(parent->scaleX, local->translation.x);
        local->scaleX = Q_6_10_MUL(parent->scaleX, local->scaleX);
    } else {
        x = local->translation.x;
    }

    if (parent->scaleY != Q_6_10(1.0)) {
        y = Q_6_10_MUL(parent->scaleY, local->translation.y);
        local->scaleY = Q_6_10_MUL(parent->scaleY, local->scaleY);
    } else {
        y = local->translation.y;
    }

    if (parent->rotation) {
        local->translation.x = parent->translation.x + (Q_MUL(x, COS_24_8(parent->rotation)) - Q_MUL(y, SIN_24_8(parent->rotation)));
        local->translation.y = parent->translation.y + (Q_MUL(x, SIN_24_8(parent->rotation)) + Q_MUL(y, COS_24_8(parent->rotation)));
    } else {
        local->translation.x = parent->translation.x + x;
        local->translation.y = parent->translation.y + y;
    }

    local->rotation = CLAMP_SIN_PERIOD(local->rotation + parent->rotation);
}

UNUSED void Transform2D_ApplyToParent(Transform2D *local, Transform2D *parent)
{
    s32 x, y;

    if (parent->scaleX != Q_6_10(1.0)) {
        x = Q_6_10_MUL(parent->scaleX, local->translation.x);
        parent->scaleX = Q_6_10_MUL(parent->scaleX, local->scaleX);
    } else {
        x = local->translation.x;
    }

    if (parent->scaleY != Q_6_10(1.0)) {
        y = Q_6_10_MUL(parent->scaleY, local->translation.y);
        parent->scaleY = Q_6_10_MUL(parent->scaleY, local->scaleY);
    } else {
        y = local->translation.y;
    }

    if (parent->rotation) {
        parent->translation.x += Q_MUL(x, COS_24_8(parent->rotation)) - Q_MUL(y, SIN_24_8(parent->rotation));
        parent->translation.y += Q_MUL(x, SIN_24_8(parent->rotation)) + Q_MUL(y, COS_24_8(parent->rotation));
    } else {
        parent->translation.x += x;
        parent->translation.y += y;
    }

    parent->rotation = CLAMP_SIN_PERIOD(local->rotation + parent->rotation);
}

UNUSED void Transform2D_Invert(Transform2D *transform)
{
    if (transform->scaleX != Q_6_10(1.0)) {
        transform->scaleX = Div(SQUARE(Q_6_10(1.0)), transform->scaleX);
        transform->translation.x = Q_6_10_MUL(transform->scaleX, transform->translation.x);
    }

    if (transform->scaleY != Q_6_10(1.0)) {
        transform->scaleY = Div(SQUARE(Q_6_10(1.0)), transform->scaleY);
        transform->translation.y = Q_6_10_MUL(transform->scaleY, transform->translation.y);
    }

    transform->rotation = CLAMP_SIN_PERIOD(SIN_PERIOD - transform->rotation);
    if (transform->rotation) {
        Vec2_32 translation = {
            Q_MUL(-transform->translation.x, COS_24_8(transform->rotation)) - Q_MUL(-transform->translation.y, SIN_24_8(transform->rotation)),
            Q_MUL(-transform->translation.x, SIN_24_8(transform->rotation)) + Q_MUL(-transform->translation.y, COS_24_8(transform->rotation)),
        };

        transform->translation.y = translation.y;
        transform->translation.x = translation.x;
    } else {
        NEGATE(transform->translation.x);
        NEGATE(transform->translation.y);
    }
}

inline void Vec3_Normalize(Vec3_16 *v, u8 n)
{
    u16 dist = Sqrt(SQUARE(v->x) + SQUARE(v->y) + SQUARE(v->z));
    s32 scale = BitValue(2 * n) / dist;

    v->x = scale * v->x >> n;
    v->y = scale * v->y >> n;
    v->z = scale * v->z >> n;
}

UNUSED inline s32 Vec3_Dot(Vec3_16 *u, Vec3_16 *v, u8 n) { return (u->x * v->x + u->y * v->y + u->z * v->z) >> n; }

inline void Vec3_Cross(Vec3_16 *u, Vec3_16 *v, Vec3_16 *out, u8 n)
{
    out->x = (u->y * v->z - u->z * v->y) >> n;
    out->y = (u->z * v->x - u->x * v->z) >> n;
    out->z = (u->x * v->y - u->y * v->x) >> n;
}

UNUSED inline void Quaternion_Init(Quaternion *quat)
{
    quat->x = quat->y = quat->z = 0;
    quat->w = Q_6_10(1.0);
}

UNUSED void Quaternion_Normalize(Quaternion *quat)
{
    s32 scale = SQUARE(Q_6_10(1.0)) / Sqrt(SQUARE(quat->x) + SQUARE(quat->y) + SQUARE(quat->z) + SQUARE(quat->w));

    quat->x = Q_6_10_MUL(quat->x, scale);
    quat->y = Q_6_10_MUL(quat->y, scale);
    quat->z = Q_6_10_MUL(quat->z, scale);
    quat->w = Q_6_10_MUL(quat->w, scale);
}

UNUSED void Quaternion_Apply(Quaternion *local, Quaternion *parent, Quaternion *out)
{
    out->x = Q_6_10_TO_INT(local->w * parent->x + local->x * parent->w + local->y * parent->z - local->z * parent->y);
    out->y = Q_6_10_TO_INT(local->w * parent->y - local->x * parent->z + local->y * parent->w + local->z * parent->x);
    out->z = Q_6_10_TO_INT(local->w * parent->z + local->x * parent->y - local->y * parent->x + local->z * parent->w);
    out->w = Q_6_10_TO_INT(local->w * parent->w - local->x * parent->x - local->y * parent->y - local->z * parent->z);
}

UNUSED void Quaternion_ApplyToLocal(Quaternion *local, Quaternion *parent)
{
    s16 x = local->x;
    s16 y = local->y;
    s16 z = local->z;

    local->x = Q_6_10_TO_INT(local->w * parent->x + x * parent->w + y * parent->z - z * parent->y);
    local->y = Q_6_10_TO_INT(local->w * parent->y - x * parent->z + y * parent->w + z * parent->x);
    local->z = Q_6_10_TO_INT(local->w * parent->z + x * parent->y - y * parent->x + z * parent->w);
    local->w = Q_6_10_TO_INT(local->w * parent->w - x * parent->x - y * parent->y - z * parent->z);
}

UNUSED void Quaternion_ApplyToParent(Quaternion *local, Quaternion *parent)
{
    s16 x = parent->x;
    s16 y = parent->y;
    s16 z = parent->z;

    parent->x = Q_6_10_TO_INT(local->w * x + local->x * parent->w + local->y * z - local->z * y);
    parent->y = Q_6_10_TO_INT(local->w * y - local->x * z + local->y * parent->w + local->z * x);
    parent->z = Q_6_10_TO_INT(local->w * z + local->x * y - local->y * x + local->z * parent->w);
    parent->w = Q_6_10_TO_INT(local->w * parent->w - local->x * x - local->y * y - local->z * z);
}

UNUSED inline void Quaternion_Conjugate(Quaternion *quat)
{
    quat->x = -quat->x;
    quat->y = -quat->y;
    quat->z = -quat->z;
}

UNUSED void Quaternion_Invert(Quaternion *quat)
{
    s32 scale = SQUARE(Q_6_10(1.0)) / Sqrt(SQUARE(quat->x) + SQUARE(quat->y) + SQUARE(quat->z) + SQUARE(quat->w));

    quat->x = Q_6_10_MUL(-quat->x, scale);
    quat->y = Q_6_10_MUL(-quat->y, scale);
    quat->z = Q_6_10_MUL(-quat->z, scale);
    quat->w = Q_6_10_MUL(quat->w, scale);
}

UNUSED inline void Quaternion_RotateX(Quaternion *quat, u16 angle)
{
    s16 angleSin;

    HALVE(angle);
    angleSin = Q_2_14_TO_Q_6_10(SIN(angle));

    quat->x = angleSin;
    quat->y = 0;
    quat->z = 0;
    quat->w = Q_2_14_TO_Q_6_10(COS(angle));
}

UNUSED inline void Quaternion_RotateY(Quaternion *quat, u16 angle)
{
    s16 angleSin; 

    HALVE(angle);
    angleSin = Q_2_14_TO_Q_6_10(SIN(angle));

    quat->x = 0;
    quat->y = angleSin;
    quat->z = 0;
    quat->w = Q_2_14_TO_Q_6_10(COS(angle));
}

UNUSED inline void Quaternion_RotateZ(Quaternion *quat, u16 angle)
{
    s16 angleSin; 

    HALVE(angle);
    angleSin = Q_2_14_TO_Q_6_10(SIN(angle));

    quat->x = 0;
    quat->y = 0;
    quat->z = angleSin;
    quat->w = Q_2_14_TO_Q_6_10(COS(angle));
}

UNUSED inline void Quaternion_RotateAxis(Quaternion *quat, u16 angle, Vec3_16 *axis)
{
    s16 angleSin;

    HALVE(angle);
    angleSin = Q_2_14_TO_Q_6_10(SIN(angle));

    quat->x = Q_6_10_MUL(angleSin, axis->x);
    quat->y = Q_6_10_MUL(angleSin, axis->y);
    quat->z = Q_6_10_MUL(angleSin, axis->z);
    quat->w = Q_2_14_TO_Q_6_10(COS(angle));
}

inline void Transform3D_Clear(Transform3D *transform)
{
#if PORTABLE
    // Modern GCC doesn't accept l-value cast increments.
    // Thankfully, we only need a memset to bypass that!
    memset(transform, 0, sizeof(Transform3D));
#else
    u8 i;

    for (i = 0; i < sizeof(Transform3D); i += 4) {
        *((u32 *)transform)++ = 0;
    }
#endif
}

inline void Transform3D_Init(Transform3D *transform)
{
    Transform3D_Clear(transform);

    transform->linear[0][0] = transform->linear[1][1] = transform->linear[2][2] = Q_6_10(1.0);
}

UNUSED inline void Transform3D_InitTranslation(Transform3D *transform, s32 x, s32 y, s32 z)
{
    Transform3D_Init(transform);

    transform->translation.v32.x = x;
    transform->translation.v32.y = y;
    transform->translation.v32.z = z;
}

UNUSED inline void Transform3D_InitIdentity(Transform3D *transform, s32 m00, s32 m11, s32 m22)
{
    Transform3D_Init(transform);

    transform->linear[0][0] = m00;
    transform->linear[1][1] = m11;
    transform->linear[2][2] = m22;
}

static void Transform3D_Apply(Transform3D *local, Transform3D *parent, Transform3D *out)
{
    out->linear[0][0] = Q_6_10_TO_INT(local->linear[0][0] * parent->linear[0][0] + local->linear[0][1] * parent->linear[1][0] + local->linear[0][2] * parent->linear[2][0]);
    out->linear[0][1] = Q_6_10_TO_INT(local->linear[0][0] * parent->linear[0][1] + local->linear[0][1] * parent->linear[1][1] + local->linear[0][2] * parent->linear[2][1]);
    out->linear[0][2] = Q_6_10_TO_INT(local->linear[0][0] * parent->linear[0][2] + local->linear[0][1] * parent->linear[1][2] + local->linear[0][2] * parent->linear[2][2]);

    out->linear[1][0] = Q_6_10_TO_INT(local->linear[1][0] * parent->linear[0][0] + local->linear[1][1] * parent->linear[1][0] + local->linear[1][2] * parent->linear[2][0]);
    out->linear[1][1] = Q_6_10_TO_INT(local->linear[1][0] * parent->linear[0][1] + local->linear[1][1] * parent->linear[1][1] + local->linear[1][2] * parent->linear[2][1]);
    out->linear[1][2] = Q_6_10_TO_INT(local->linear[1][0] * parent->linear[0][2] + local->linear[1][1] * parent->linear[1][2] + local->linear[1][2] * parent->linear[2][2]);

    out->linear[2][0] = Q_6_10_TO_INT(local->linear[2][0] * parent->linear[0][0] + local->linear[2][1] * parent->linear[1][0] + local->linear[2][2] * parent->linear[2][0]);
    out->linear[2][1] = Q_6_10_TO_INT(local->linear[2][0] * parent->linear[0][1] + local->linear[2][1] * parent->linear[1][1] + local->linear[2][2] * parent->linear[2][1]);
    out->linear[2][2] = Q_6_10_TO_INT(local->linear[2][0] * parent->linear[0][2] + local->linear[2][1] * parent->linear[1][2] + local->linear[2][2] * parent->linear[2][2]);

    out->translation.v32.x = Q_6_10_TO_INT((s64)local->translation.v32.x * parent->linear[0][0] + (s64)local->translation.v32.y * parent->linear[1][0] + (s64)local->translation.v32.z * parent->linear[2][0] + parent->translation.v32.x);
    out->translation.v32.y = Q_6_10_TO_INT((s64)local->translation.v32.x * parent->linear[0][1] + (s64)local->translation.v32.y * parent->linear[1][1] + (s64)local->translation.v32.z * parent->linear[2][1] + parent->translation.v32.y);
    out->translation.v32.z = Q_6_10_TO_INT((s64)local->translation.v32.x * parent->linear[0][2] + (s64)local->translation.v32.y * parent->linear[1][2] + (s64)local->translation.v32.z * parent->linear[2][2] + parent->translation.v32.z);
}

UNUSED void Transform3D_ApplyToLocal(Transform3D *local, Transform3D *parent)
{
    s32 x, y, z;

    x = local->linear[0][0];
    y = local->linear[0][1];
    z = local->linear[0][2];
    local->linear[0][0] = Q_6_10_TO_INT(x * parent->linear[0][0] + y * parent->linear[1][0] + z * parent->linear[2][0]);
    local->linear[0][1] = Q_6_10_TO_INT(x * parent->linear[0][1] + y * parent->linear[1][1] + z * parent->linear[2][1]);
    local->linear[0][2] = Q_6_10_TO_INT(x * parent->linear[0][2] + y * parent->linear[1][2] + z * parent->linear[2][2]);

    x = local->linear[1][0];
    y = local->linear[1][1];
    z = local->linear[1][2];
    local->linear[1][0] = Q_6_10_TO_INT(x * parent->linear[0][0] + y * parent->linear[1][0] + z * parent->linear[2][0]);
    local->linear[1][1] = Q_6_10_TO_INT(x * parent->linear[0][1] + y * parent->linear[1][1] + z * parent->linear[2][1]);
    local->linear[1][2] = Q_6_10_TO_INT(x * parent->linear[0][2] + y * parent->linear[1][2] + z * parent->linear[2][2]);

    x = local->linear[2][0];
    y = local->linear[2][1];
    z = local->linear[2][2];
    local->linear[2][0] = Q_6_10_TO_INT(x * parent->linear[0][0] + y * parent->linear[1][0] + z * parent->linear[2][0]);
    local->linear[2][1] = Q_6_10_TO_INT(x * parent->linear[0][1] + y * parent->linear[1][1] + z * parent->linear[2][1]);
    local->linear[2][2] = Q_6_10_TO_INT(x * parent->linear[0][2] + y * parent->linear[1][2] + z * parent->linear[2][2]);

    x = local->translation.v32.x;
    y = local->translation.v32.y;
    z = local->translation.v32.z;
    local->translation.v32.x = Q_6_10_TO_INT((s64)x * parent->linear[0][0] + (s64)y * parent->linear[1][0] + (s64)z * parent->linear[2][0] + parent->translation.v32.x);
    local->translation.v32.y = Q_6_10_TO_INT((s64)x * parent->linear[0][1] + (s64)y * parent->linear[1][1] + (s64)z * parent->linear[2][1] + parent->translation.v32.y);
    local->translation.v32.z = Q_6_10_TO_INT((s64)x * parent->linear[0][2] + (s64)y * parent->linear[1][2] + (s64)z * parent->linear[2][2] + parent->translation.v32.z);
}

UNUSED void Transform3D_ApplyToParent(Transform3D *local, Transform3D *parent)
{
    s32 x, y, z;

    parent->translation.v32.x = Q_6_10_TO_INT((s64)local->translation.v32.x * parent->linear[0][0] + (s64)local->translation.v32.y * parent->linear[1][0] + (s64)local->translation.v32.z * parent->linear[2][0] + parent->translation.v32.x);
    parent->translation.v32.y = Q_6_10_TO_INT((s64)local->translation.v32.x * parent->linear[0][1] + (s64)local->translation.v32.y * parent->linear[1][1] + (s64)local->translation.v32.z * parent->linear[2][1] + parent->translation.v32.y);
    parent->translation.v32.z = Q_6_10_TO_INT((s64)local->translation.v32.x * parent->linear[0][2] + (s64)local->translation.v32.y * parent->linear[1][2] + (s64)local->translation.v32.z * parent->linear[2][2] + parent->translation.v32.z);

    x = parent->linear[0][0];
    y = parent->linear[1][0];
    z = parent->linear[2][0];
    parent->linear[0][0] = Q_6_10_TO_INT(local->linear[0][0] * x + local->linear[0][1] * y + local->linear[0][2] * z);
    parent->linear[1][0] = Q_6_10_TO_INT(local->linear[1][0] * x + local->linear[1][1] * y + local->linear[1][2] * z);
    parent->linear[2][0] = Q_6_10_TO_INT(local->linear[2][0] * x + local->linear[2][1] * y + local->linear[2][2] * z);

    x = parent->linear[0][1];
    y = parent->linear[1][1];
    z = parent->linear[2][1];
    parent->linear[0][1] = Q_6_10_TO_INT(local->linear[0][0] * x + local->linear[0][1] * y + local->linear[0][2] * z);
    parent->linear[1][1] = Q_6_10_TO_INT(local->linear[1][0] * x + local->linear[1][1] * y + local->linear[1][2] * z);
    parent->linear[2][1] = Q_6_10_TO_INT(local->linear[2][0] * x + local->linear[2][1] * y + local->linear[2][2] * z);

    x = parent->linear[0][2];
    y = parent->linear[1][2];
    z = parent->linear[2][2];
    parent->linear[0][2] = Q_6_10_TO_INT(local->linear[0][0] * x + local->linear[0][1] * y + local->linear[0][2] * z);
    parent->linear[1][2] = Q_6_10_TO_INT(local->linear[1][0] * x + local->linear[1][1] * y + local->linear[1][2] * z);
    parent->linear[2][2] = Q_6_10_TO_INT(local->linear[2][0] * x + local->linear[2][1] * y + local->linear[2][2] * z);
}

UNUSED void Transform3D_PointTransform(Vec3_32 *v, Transform3D *transform, Vec3_32 *out)
{
    out->x = Q_6_10_TO_INT((s64)v->x * transform->linear[0][0] + (s64)v->y * transform->linear[1][0] + (s64)v->z * transform->linear[2][0] + transform->translation.v32.x);
    out->y = Q_6_10_TO_INT((s64)v->x * transform->linear[0][1] + (s64)v->y * transform->linear[1][1] + (s64)v->z * transform->linear[2][1] + transform->translation.v32.y);
    out->z = Q_6_10_TO_INT((s64)v->x * transform->linear[0][2] + (s64)v->y * transform->linear[1][2] + (s64)v->z * transform->linear[2][2] + transform->translation.v32.z);
}

UNUSED void Transform3D_PointTransformIP(Vec3_32 *v, Transform3D *transform)
{
#ifdef BUGFIX
    s32 x = v->x;
    s32 y = v->y;

    v->x = Q_6_10_TO_INT((s64)x * transform->linear[0][0] + (s64)y * transform->linear[1][0] + (s64)v->z * transform->linear[2][0] + transform->translation.v32.x);
    v->y = Q_6_10_TO_INT((s64)x * transform->linear[0][1] + (s64)y * transform->linear[1][1] + (s64)v->z * transform->linear[2][1] + transform->translation.v32.y);
    v->z = Q_6_10_TO_INT((s64)x * transform->linear[0][2] + (s64)y * transform->linear[1][2] + (s64)v->z * transform->linear[2][2] + transform->translation.v32.z);
#else
    // This uses the already modified values rather than
    // the originals for the y and z transforms
    v->x = Q_6_10_TO_INT((s64)v->x * transform->linear[0][0] + (s64)v->y * transform->linear[1][0] + (s64)v->z * transform->linear[2][0] + transform->translation.v32.x);
    v->y = Q_6_10_TO_INT((s64)v->x * transform->linear[0][1] + (s64)v->y * transform->linear[1][1] + (s64)v->z * transform->linear[2][1] + transform->translation.v32.y);
    v->z = Q_6_10_TO_INT((s64)v->x * transform->linear[0][2] + (s64)v->y * transform->linear[1][2] + (s64)v->z * transform->linear[2][2] + transform->translation.v32.z);
#endif
}

UNUSED void Transform3D_PointProject(Transform3D *transform, Vec3_32 *v, Vec3_32 *out)
{
    s32 scale = SQUARE(Q_6_10(1.0)) / Q_6_10_TO_INT((s64)v->x * transform->translation.v32.x + (s64)v->y * transform->translation.v32.y + (s64)v->z * transform->translation.v32.z + SQUARE(Q_6_10(1.0)));

    out->x = Q_6_10_MUL(Q_6_10_TO_INT((s64)v->x * transform->linear[0][0] + (s64)v->y * transform->linear[0][1] + (s64)v->z * transform->linear[0][2]), scale);
    out->y = Q_6_10_MUL(Q_6_10_TO_INT((s64)v->x * transform->linear[1][0] + (s64)v->y * transform->linear[1][1] + (s64)v->z * transform->linear[1][2]), scale);
    out->z = Q_6_10_MUL(Q_6_10_TO_INT((s64)v->x * transform->linear[2][0] + (s64)v->y * transform->linear[2][1] + (s64)v->z * transform->linear[2][2]), scale);
}

UNUSED void Transform3D_PointProjectIP(Transform3D *transform, Vec3_32 *v)
{
    s32 x = v->x;
    s32 y = v->y;

    s32 scale = SQUARE(Q_6_10(1.0)) / Q_6_10_TO_INT(x * transform->translation.v32.x + y * transform->translation.v32.y + v->z * transform->translation.v32.z + SQUARE(Q_6_10(1.0)));

    v->x = Q_6_10_MUL(Q_6_10_TO_INT((s64)x * transform->linear[0][0] + (s64)y * transform->linear[0][1] + (s64)v->z * transform->linear[0][2]), scale);
    v->y = Q_6_10_MUL(Q_6_10_TO_INT((s64)x * transform->linear[1][0] + (s64)y * transform->linear[1][1] + (s64)v->z * transform->linear[1][2]), scale);
    v->z = Q_6_10_MUL(Q_6_10_TO_INT((s64)x * transform->linear[2][0] + (s64)y * transform->linear[2][1] + (s64)v->z * transform->linear[2][2]), scale);
}

UNUSED void Transform3D_RotateX(Transform3D *transform, u16 angle)
{
    s16 angleSin, angleCos;

    Transform3D_Init(transform);

    angleSin = Q_2_14_TO_Q_6_10(SIN(angle));
    angleCos = Q_2_14_TO_Q_6_10(COS(angle));

    transform->linear[1][1] = angleCos;
    transform->linear[1][2] = angleSin;
    transform->linear[2][1] = -angleSin;
    transform->linear[2][2] = angleCos;
}

UNUSED void Transform3D_RotateY(Transform3D *transform, u16 angle)
{
    s16 angleSin, angleCos;

    Transform3D_Init(transform);

    angleSin = Q_2_14_TO_Q_6_10(SIN(angle));
    angleCos = Q_2_14_TO_Q_6_10(COS(angle));

    transform->linear[0][0] = angleCos;
    transform->linear[0][2] = -angleSin;
    transform->linear[2][0] = angleSin;
    transform->linear[2][2] = angleCos;
}

UNUSED void Transform3D_RotateZ(Transform3D *transform, u16 angle)
{
    s16 angleSin, angleCos;

    Transform3D_Init(transform);

    angleSin = Q_2_14_TO_Q_6_10(SIN(angle));
    angleCos = Q_2_14_TO_Q_6_10(COS(angle));

    transform->linear[0][0] = angleCos;
    transform->linear[0][1] = angleSin;
    transform->linear[1][0] = -angleSin;
    transform->linear[1][1] = angleCos;
}

UNUSED void Transform3D_RotateAxis(Transform3D *transform, u16 angle, Vec3_16 *axis)
{
    s16 angleSin, xy, yz, xz;
    s32 angleCos;

    Transform3D_Init(transform);

    angleSin = Q_2_14_TO_Q_6_10(SIN(angle));
    angleCos = Q_2_14_TO_Q_6_10(COS(angle));

    xy = Q_6_10_MUL(Q_6_10_MUL(axis->x, axis->y), Q_6_10(1.0) - angleCos);
    yz = Q_6_10_MUL(Q_6_10_MUL(axis->y, axis->z), Q_6_10(1.0) - angleCos);
    xz = Q_6_10_MUL(Q_6_10_MUL(axis->x, axis->z), Q_6_10(1.0) - angleCos);

    transform->linear[0][0] = angleCos + Q_6_10_MUL(Q_6_10_SQUARE(axis->x), Q_6_10(1.0) - angleCos);
    transform->linear[0][1] = xy + Q_6_10_MUL(axis->z, angleSin);
    transform->linear[0][2] = xz - Q_6_10_MUL(axis->y, angleSin);

    transform->linear[1][0] = xy - Q_6_10_MUL(axis->z, angleSin);
    transform->linear[1][1] = angleCos + Q_6_10_MUL(Q_6_10_SQUARE(axis->y), Q_6_10(1.0) - angleCos);
    transform->linear[1][2] = yz + Q_6_10_MUL(axis->x, angleSin);

    transform->linear[2][0] = xz + Q_6_10_MUL(axis->y, angleSin);
    transform->linear[2][1] = yz - Q_6_10_MUL(axis->x, angleSin);
    transform->linear[2][2] = angleCos + Q_6_10_MUL(Q_6_10_SQUARE(axis->z), Q_6_10(1.0) - angleCos);
}

static void Transform3D_RotateXYZ(Transform3D *transform, u16 angleX, u16 angleY, u16 angleZ)
{
    s16 ySin = Q_2_14_TO_Q_6_10(SIN(angleY));
    s16 yCos = Q_2_14_TO_Q_6_10(COS(angleY));

    s16 xSin = Q_2_14_TO_Q_6_10(SIN(angleX));
    s16 xCos = Q_2_14_TO_Q_6_10(COS(angleX));

    s16 zSin = Q_2_14_TO_Q_6_10(SIN(angleZ));
    s16 zCos = Q_2_14_TO_Q_6_10(COS(angleZ));

    transform->linear[0][0] = Q_6_10_MUL(zCos, xCos) + Q_6_10_MUL(Q_6_10_MUL(zSin, ySin), xSin);
    transform->linear[0][1] = Q_6_10_MUL(zSin, yCos);
    transform->linear[0][2] = Q_6_10_MUL(-zCos, xSin) + Q_6_10_MUL(Q_6_10_MUL(zSin, ySin), xCos);

    transform->linear[1][0] = Q_6_10_MUL(-zSin, xCos) + Q_6_10_MUL(xSin, Q_6_10_MUL(ySin, zCos));
    transform->linear[1][1] = Q_6_10_MUL(zCos, yCos);
    transform->linear[1][2] = Q_6_10_MUL(xSin, zSin) + Q_6_10_MUL(xCos, Q_6_10_MUL(ySin, zCos));

    transform->linear[2][0] = Q_6_10_MUL(yCos, xSin);
    transform->linear[2][1] = -ySin;
    transform->linear[2][2] = Q_6_10_MUL(yCos, xCos);

    transform->translation.v32.x = transform->translation.v32.y = transform->translation.v32.z = 0;
}

UNUSED void Transform3D_LookAt(Transform3D *transform, Vec3_32 *pos, Vec3_32 *target, Vec3_16 *originY)
{
    transform->translation.v32.x = pos->x;
    transform->translation.v32.y = pos->y;
    transform->translation.v32.z = pos->z;

    Vec3_Sub(target, pos, (Vec3_16 *)transform->linear[2]);
#if BUGFIX
    if (Vec3_Dot((Vec3_16 *)transform->linear[2], (Vec3_16 *)transform->linear[2], 10) > 0) {
#else
    // This checks for the absolute component sum, rather than the squared magnitude.
    // Basically, it means valid directions can be interpreted as degenerate, which is no good!
    if (ABS(transform->linear[2][0] + transform->linear[2][1] + transform->linear[2][2]) > 32) {
#endif
        Vec3_Normalize((Vec3_16 *)transform->linear[2], 10);
    } else {
        transform->linear[2][0] = 0;
        transform->linear[2][1] = 0;
        transform->linear[2][2] = Q_6_10(1.0);
    }

    Vec3_Cross(originY, (Vec3_16 *)transform->linear[2], (Vec3_16 *)transform->linear[0], 10);
#if BUGFIX
    if (Vec3_Dot((Vec3_16 *)transform->linear[0], (Vec3_16 *)transform->linear[0], 10) > 0) {
#else
    if (ABS(transform->linear[0][0] + transform->linear[0][1] + transform->linear[0][2]) > 32) {
#endif
        Vec3_Normalize((Vec3_16 *)transform->linear[0], 10);
    } else {
        transform->linear[0][0] = Q_6_10(1.0);
        transform->linear[0][1] = 0;
        transform->linear[0][2] = 0;
    }

    Vec3_Cross((Vec3_16 *)transform->linear[2], (Vec3_16 *)transform->linear[0], (Vec3_16 *)transform->linear[1], 10);
#if BUGFIX
    if (Vec3_Dot((Vec3_16 *)transform->linear[1], (Vec3_16 *)transform->linear[1], 10) > 0) {
#else
    if (ABS(transform->linear[1][0] + transform->linear[1][1] + transform->linear[1][2]) > 32) {
#endif
        Vec3_Normalize((Vec3_16 *)transform->linear[1], 10);
    } else {
        transform->linear[1][0] = 0;
        transform->linear[1][1] = Q_6_10(1.0);
        transform->linear[1][2] = 0;
    }
}

static void Transform3D_Transpose(Transform3D *transform)
{
    XOR_SWAP_2(transform->linear[0][1], transform->linear[1][0]);
    XOR_SWAP_2(transform->linear[0][2], transform->linear[2][0]);
    XOR_SWAP_2(transform->linear[1][2], transform->linear[2][1]);
}

UNUSED void Transform3D_Invert(Transform3D *transform)
{
    s32 scale = SQUARE(Q_6_10(1.0)) / (transform->linear[0][0] * Q_6_10_TO_INT(transform->linear[1][1] * transform->linear[2][2] - transform->linear[1][2] * transform->linear[2][1]) - transform->linear[0][1] * Q_6_10_TO_INT(transform->linear[1][0] * transform->linear[2][2] - transform->linear[1][2] * transform->linear[2][0]) + transform->linear[0][2] * Q_6_10_TO_INT(transform->linear[1][0] * transform->linear[2][1] - transform->linear[1][1] * transform->linear[2][0]));

    Transform3D tmpTransform = *transform;

    transform->linear[0][0] = Q_6_10_MUL(scale, Q_6_10_TO_INT(tmpTransform.linear[1][1] * tmpTransform.linear[2][2] - tmpTransform.linear[1][2] * tmpTransform.linear[2][1]));
    transform->linear[0][1] = Q_6_10_MUL(-scale, Q_6_10_TO_INT(tmpTransform.linear[0][1] * tmpTransform.linear[2][2] - tmpTransform.linear[0][2] * tmpTransform.linear[2][1]));
    transform->linear[0][2] = Q_6_10_MUL(scale, Q_6_10_TO_INT(tmpTransform.linear[0][1] * tmpTransform.linear[1][2] - tmpTransform.linear[0][2] * tmpTransform.linear[1][1]));
    transform->linear[1][0] = Q_6_10_MUL(-scale, Q_6_10_TO_INT(tmpTransform.linear[1][0] * tmpTransform.linear[2][2] - tmpTransform.linear[1][2] * tmpTransform.linear[2][0]));
    transform->linear[1][1] = Q_6_10_MUL(scale, Q_6_10_TO_INT(tmpTransform.linear[0][0] * tmpTransform.linear[2][2] - tmpTransform.linear[0][2] * tmpTransform.linear[2][0]));
    transform->linear[1][2] = Q_6_10_MUL(-scale, Q_6_10_TO_INT(tmpTransform.linear[0][0] * tmpTransform.linear[1][2] - tmpTransform.linear[0][2] * tmpTransform.linear[1][0]));
    transform->linear[2][0] = Q_6_10_MUL(scale, Q_6_10_TO_INT(tmpTransform.linear[1][0] * tmpTransform.linear[2][1] - tmpTransform.linear[1][1] * tmpTransform.linear[2][0]));
    transform->linear[2][1] = Q_6_10_MUL(-scale, Q_6_10_TO_INT(tmpTransform.linear[0][0] * tmpTransform.linear[2][1] - tmpTransform.linear[0][1] * tmpTransform.linear[2][0]));
    transform->linear[2][2] = Q_6_10_MUL(scale, Q_6_10_TO_INT(tmpTransform.linear[0][0] * tmpTransform.linear[1][1] - tmpTransform.linear[0][1] * tmpTransform.linear[1][0]));

#if BUGFIX
    transform->translation.v32.x = Q_6_10_TO_INT(-((s64)tmpTransform.translation.v32.x * tmpTransform.linear[0][0] + (s64)tmpTransform.translation.v32.y * tmpTransform.linear[1][0] + (s64)tmpTransform.translation.v32.z * tmpTransform.linear[2][0]));
    transform->translation.v32.y = Q_6_10_TO_INT(-((s64)tmpTransform.translation.v32.x * tmpTransform.linear[0][1] + (s64)tmpTransform.translation.v32.y * tmpTransform.linear[1][1] + (s64)tmpTransform.translation.v32.z * tmpTransform.linear[2][1]));
    transform->translation.v32.z = Q_6_10_TO_INT(-((s64)tmpTransform.translation.v32.x * tmpTransform.linear[0][2] + (s64)tmpTransform.translation.v32.y * tmpTransform.linear[1][2] + (s64)tmpTransform.translation.v32.z * tmpTransform.linear[2][2]));
#else
    // I presume this is a leftover from when Transform3D used Vec3_16 for it's translation?
    // If not, though... then why the hell don't you use the 32-bit version too???
    transform->translation.v16.x = Q_6_10_TO_INT(-((s64)tmpTransform.translation.v32.x * tmpTransform.linear[0][0] + (s64)tmpTransform.translation.v32.y * tmpTransform.linear[1][0] + (s64)tmpTransform.translation.v32.z * tmpTransform.linear[2][0]));
    transform->translation.v16.y = Q_6_10_TO_INT(-((s64)tmpTransform.translation.v32.x * tmpTransform.linear[0][1] + (s64)tmpTransform.translation.v32.y * tmpTransform.linear[1][1] + (s64)tmpTransform.translation.v32.z * tmpTransform.linear[2][1]));
    transform->translation.v16.z = Q_6_10_TO_INT(-((s64)tmpTransform.translation.v32.x * tmpTransform.linear[0][2] + (s64)tmpTransform.translation.v32.y * tmpTransform.linear[1][2] + (s64)tmpTransform.translation.v32.z * tmpTransform.linear[2][2]));
#endif
}

UNUSED void Transform3D_InvertRigid(Transform3D *transform)
{
    s32 x, y;

    Transform3D_Transpose(transform);

    x = NEGATE(transform->translation.v32.x);
    y = NEGATE(transform->translation.v32.y);
        NEGATE(transform->translation.v32.z);

    transform->translation.v32.x = Q_6_10_TO_INT((s64)x * transform->linear[0][0] + (s64)y * transform->linear[1][0] + (s64)transform->translation.v32.z * transform->linear[2][0]);
    transform->translation.v32.y = Q_6_10_TO_INT((s64)x * transform->linear[0][1] + (s64)y * transform->linear[1][1] + (s64)transform->translation.v32.z * transform->linear[2][1]);
    transform->translation.v32.z = Q_6_10_TO_INT((s64)x * transform->linear[0][2] + (s64)y * transform->linear[1][2] + (s64)transform->translation.v32.z * transform->linear[2][2]);
}

UNUSED void Transform3D_FromQuaternion(Transform3D *transform, Quaternion *quat)
{
    s32 x2 = quat->x * 2;
    s32 y2 = quat->y * 2;
    s32 z2 = quat->z * 2;

    s16 x2W = Q_6_10_MUL(x2, quat->w);
    s16 y2W = Q_6_10_MUL(y2, quat->w);
    s16 z2W = Q_6_10_MUL(z2, quat->w);

    s16 x = quat->x;
    s16 x2X = Q_6_10_MUL(x2, x);
    s16 y2X = Q_6_10_MUL(y2, x);
    s16 z2X = Q_6_10_MUL(z2, x);

    s16 y = quat->y;
    s16 y2Y = Q_6_10_MUL(y2, y);
    s16 z2Y = Q_6_10_MUL(z2, y);

    s16 z = quat->z;
    s16 z2Z = Q_6_10_MUL(z2, z);

    transform->linear[0][0] = Q_6_10(1.0) - (y2Y + z2Z);
    transform->linear[0][1] = y2X - z2W;
    transform->linear[0][2] = z2X + y2W;

    transform->linear[1][0] = y2X + z2W;
    transform->linear[1][1] = Q_6_10(1.0) - (x2X + z2Z);
    transform->linear[1][2] = z2Y - x2W;

    transform->linear[2][0] = z2X - y2W;
    transform->linear[2][1] = z2Y + x2W;
    transform->linear[2][2] = Q_6_10(1.0) - (x2X + y2Y);

    transform->translation.v32.x = transform->translation.v32.y = transform->translation.v32.z = 0;
}

UNUSED inline void TransformNode_Init(TransformNode *node)
{
    node->parent = NULL;
    node->child = NULL;
    node->next = NULL;
    node->flags = TRANSFORM_NONE;
}

UNUSED inline TransformNode *TransformNode_GetRoot(TransformNode *node)
{
    for (; node->parent != NULL; node = node->parent);

    return node;
}

inline void TransformNode_DetachFromParent(TransformNode *node)
{
    if (node->parent != NULL) {
        bool8 removed = TransformNode_RemoveChild(node->parent, node);
#if DEBUG
        assert(removed);
#endif
    }
}

static inline bool8 TransformNode_RemoveChild(TransformNode *parent, TransformNode *child)
{
    TransformNode *curChild, *prevChild;

    for (curChild = parent->child, prevChild = NULL; curChild != NULL; prevChild = curChild, curChild = curChild->next) {
        if (curChild == child) {
            if (prevChild == NULL) {
                parent->child = curChild->next;
            } else {
                prevChild->next = curChild->next;
            }

            child->parent = NULL;
            child->next = NULL;

            return TRUE;
        }
    }

    return FALSE;
}

UNUSED inline void TransformNode_AddChild(TransformNode *parent, TransformNode *child)
{
    TransformNode_DetachFromParent(child);

    child->parent = parent;
    child->next = parent->child;
    parent->child = child;
}

UNUSED inline u16 TransformNode_GetChildCount(TransformNode *node)
{
    u16 cnt;
    for (cnt = 0, node = node->child; node != NULL; cnt++, node = node->next);

    return cnt;
}

UNUSED inline TransformNode *TransformNode_GetChildByIdx(TransformNode *node, u16 idx)
{
    for (node = node->child; node != NULL && idx != 0; idx--, node = node->next);

    return node;
}

inline void TransformNode_GetLocal2D(TransformNode *node, Transform2D *transform) { memcpy(transform, &node->transform.transform2D, sizeof(Transform2D)); }

UNUSED inline void TransformNode_SetLocal2D(TransformNode *node, Transform2D *transform) { memcpy(&node->transform.transform2D, transform, sizeof(Transform2D)); }

UNUSED inline void TransformNode_GetWorld2D(TransformNode *node, Transform2D *transform)
{
    TransformNode_GetLocal2D(node, transform);

    while ((node = node->parent) != NULL) {
        Transform2D_ApplyToLocal(transform, &node->transform.transform2D);
    }
}

UNUSED inline void TransformNode_InitLocal3D(TransformNode *node)
{
    node->transform.state3D.useTRS = FALSE;

    Transform3D_Init(&node->transform.state3D.local);
    node->flags |= TRANSFORM_LOCAL_VALID;
}

UNUSED inline void TransformNode_InitTRS(TransformNode *node)
{
    node->transform.state3D.useTRS = TRUE;

    node->transform.state3D.rotation.x = node->transform.state3D.rotation.y = node->transform.state3D.rotation.z = 0;
    node->transform.state3D.translation.x = node->transform.state3D.translation.y = node->transform.state3D.translation.z = 0;
    node->transform.state3D.scale.x = node->transform.state3D.scale.y = node->transform.state3D.scale.z = Q_6_10(1.0);
}

static Transform3D *TransformNode_GetLocal3D(TransformNode *node)
{
    if (!(node->flags & TRANSFORM_LOCAL_VALID)) {
        if (node->transform.state3D.useTRS == TRUE) {
            Transform3D_RotateXYZ(&node->transform.state3D.local, node->transform.state3D.rotation.x, node->transform.state3D.rotation.y, node->transform.state3D.rotation.z);

            node->transform.state3D.local.linear[0][0] = Q_6_10_MUL(node->transform.state3D.scale.x, node->transform.state3D.local.linear[0][0]);
            node->transform.state3D.local.linear[0][1] = Q_6_10_MUL(node->transform.state3D.scale.x, node->transform.state3D.local.linear[0][1]);
            node->transform.state3D.local.linear[0][2] = Q_6_10_MUL(node->transform.state3D.scale.x, node->transform.state3D.local.linear[0][2]);

            node->transform.state3D.local.linear[1][0] = Q_6_10_MUL(node->transform.state3D.scale.y, node->transform.state3D.local.linear[1][0]);
            node->transform.state3D.local.linear[1][1] = Q_6_10_MUL(node->transform.state3D.scale.y, node->transform.state3D.local.linear[1][1]);
            node->transform.state3D.local.linear[1][2] = Q_6_10_MUL(node->transform.state3D.scale.y, node->transform.state3D.local.linear[1][2]);

            node->transform.state3D.local.linear[2][0] = Q_6_10_MUL(node->transform.state3D.scale.z, node->transform.state3D.local.linear[2][0]);
            node->transform.state3D.local.linear[2][1] = Q_6_10_MUL(node->transform.state3D.scale.z, node->transform.state3D.local.linear[2][1]);
            node->transform.state3D.local.linear[2][2] = Q_6_10_MUL(node->transform.state3D.scale.z, node->transform.state3D.local.linear[2][2]);
        }

        node->flags |= TRANSFORM_LOCAL_VALID;
    }

    return &node->transform.state3D.local;
}

UNUSED inline Transform3D *TransformNode_UpdateWorld3D(TransformNode *node)
{
    Transform3D *parent = NULL;

    if (node->parent != NULL) {
#ifdef BUGFIX
        parent = TransformNode_UpdateWorld3D(node->parent);
#else
        // Infinite recursion, basically
        // Besides, why are you calling your cousin instead of yourself???
        parent = TransformNode_GetWorld3D(node);
#endif
    }

    if ((node->flags & TRANSFORM_ALL) == TRANSFORM_ALL) {
        if (parent != NULL) {
            Transform3D_Apply(&node->transform.state3D.local, parent, &node->transform.state3D.world);
        }
    } else {
        if (!(node->flags & TRANSFORM_LOCAL_VALID)) {
            TransformNode_GetLocal3D(node);

            node->flags |= TRANSFORM_LOCAL_VALID;
            node->flags &= ~TRANSFORM_WORLD_VALID;
        }

        Transform3D_Apply(&node->transform.state3D.local, parent, &node->transform.state3D.world);
    }

    return &node->transform.state3D.world;
}

inline Transform3D *TransformNode_GetWorld3D(TransformNode *node)
{
    Transform3D *parent = NULL;

    if (node->parent != NULL) {
#ifdef BUGFIX
        parent = TransformNode_GetWorld3D(node->parent);
#else
        // Read the comment at TransformNode_UpdateWorld3D
        parent = TransformNode_GetWorld3D(node);
#endif
    }

    if ((node->flags & TRANSFORM_ALL) == TRANSFORM_ALL) {
        if (parent == NULL) {
            return NULL;
        }
    } else if (!(node->flags & TRANSFORM_LOCAL_VALID)) {
        TransformNode_GetLocal3D(node);

        node->flags |= TRANSFORM_LOCAL_VALID;
        node->flags &= ~TRANSFORM_WORLD_VALID;
    }

    Transform3D_Apply(&node->transform.state3D.local, parent, &node->transform.state3D.world);

    return &node->transform.state3D.world;
}

UNUSED inline void Math_DummyFunc(void) { }
