#ifndef GUARD_MATH_H
#define GUARD_MATH_H

#include "global.h"

#define RAND_CONST 0x37119371

// Converts a number to Q6.10 fixed-point format
#define Q_6_10(n) ((s16)((n) * 1024))

// Converts a Q6.10 fixed-point format number to a regular integer
#define Q_6_10_TO_INT(n) ((n) >> 10)

// Converts a Q2.14 fixed-point format number to a Q6.10 fixed point number
#define Q_2_14_TO_Q_6_10(n) ((s16)((n) >> 4))

#define Q_6_10_MUL(qValA, qValB) (Q_6_10_TO_INT(qValA * qValB))
#define Q_6_10_SQUARE(qValA)     (Q_6_10_MUL(qValA, qValA))

typedef enum {
    TRANSFORM_NONE = 0,
    TRANSFORM_LOCAL_VALID = (1 << 0),
    TRANSFORM_WORLD_VALID = (1 << 1),

    TRANSFORM_ALL         = TRANSFORM_LOCAL_VALID | TRANSFORM_WORLD_VALID
} TransformNodeFlags;

typedef struct {
    s16 x;
    s16 y;
    s16 z;
} Vec3_16;

typedef struct {
    s32 x;
    s32 y;
    s32 z;
} Vec3_32;

// NOTE: This only exists because of a bug(?)
//      in Transform3D_Invert, thanks SEGA/Dimps!
typedef union {
    Vec3_16 v16;
    Vec3_32 v32;
} Vec3;

typedef struct {
    s16 x;
    s16 y;
    s16 z;
    s16 w;
} Quaternion;

typedef struct {
    u16 rotation;
    s16 scaleX;
    s16 scaleY;
    Vec2_32 translation;
} Transform2D;

typedef struct {
    u16 rotation; // Transform2D leftover
    s16 linear[3][3];
    Vec3 translation;
} Transform3D;

typedef struct {
    bool32 useTRS;

    Transform3D world;
    Transform3D local;

    Vec3_16 rotation;
    Vec3_16 scale;
    Vec3_32 translation;
} TransformState3D;

typedef union {
    Transform2D transform2D;
    TransformState3D state3D;
} TransformNodeTransform;

typedef struct TransformNode TransformNode;

struct TransformNode {
    TransformNode *parent;
    TransformNode *child;
    TransformNode *next;
    TransformNodeFlags flags;
    TransformNodeTransform transform;
};

inline u16 Random(void);
inline void SeedRng(u32 prev, u32 cur);

// Returns a 32-bit pseudorandom number
#define Random32() ((Random() << 16) | Random())

#define SHUFFLE_RNG_SEED()                                                                                                                 \
    ({                                                                                                                                     \
        u32 rand1, rand2, rand3;                                                                                                           \
        rand1 ^= (Random() << 16);                                                                                                         \
        rand1 ^= Random();                                                                                                                 \
        rand3 ^= rand1;                                                                                                                    \
        SeedRng(rand2, rand3);                                                                                                             \
        rand1 ^= (Random() << 16);                                                                                                         \
        rand1 ^= Random();                                                                                                                 \
        rand2 ^= rand1;                                                                                                                    \
        SeedRng(rand2, rand3);                                                                                                             \
    })

UNUSED inline u32 MaxAbs(s32 a, s32 b);
inline u32 ApproxVecLen(s32 a, s32 b);
inline s16 AngleDiff(u16 angleA, u16 angleB, u8 n);

inline u32 Lerp(s32 start, s32 end, s32 t, u8 n);
UNUSED inline u16 LerpAngle(u16 angleStart, u16 angleEnd, u8 n1, s16 t, u8 n2);
inline u32 LerpToEnd(s32 start, s32 end, s32 t, u8 n, u8 cnt);
inline u32 LerpToStart(s32 start, s32 end, s32 t, u8 n, u8 cnt);

UNUSED inline s32 WeightedLeft(s32 prev, s32 cur, s32 next);
UNUSED inline s32 WeightedRight(s32 prev, s32 cur, s32 next);
UNUSED inline s32 WeightedLeft2(s32 prev, s32 cur, s32 next);
UNUSED inline s32 WeightedRight2(s32 prev, s32 cur, s32 next);

inline s32 DeltaScaled(s32 start, s32 end, s32 t, UNUSED s32 adjust, UNUSED s32 offset, u8 n);
inline s32 Delta(s32 start, s32 end);

inline s32 CurveOffsetScaled(s32 prev, s32 cur, s32 next, s32 t, UNUSED s32 adjust, UNUSED s32 offset, u8 n);
inline s32 CurveOffset(s32 prev, s32 cur, s32 next);
UNUSED inline s32 CurveOffsetScaled2(s32 prev, s32 cur, s32 next, s32 t, UNUSED s32 adjust, UNUSED s32 offset, u8 n);
UNUSED inline s32 CurveOffset2(s32 prev, s32 cur, s32 next);

UNUSED inline s32 DeltaCurveOffsetScaled(s32 prev, s32 cur, s32 next, s32 t, s32 adjust, s32 offset, u8 n);
UNUSED inline s32 DeltaCurveOffsetScaledInverse(s32 prev, s32 cur, s32 next, s32 t, s32 adjust, s32 offset, u8 n);

UNUSED inline s32 DeltaCentral(s32 prev, s32 cur, s32 next);

inline s16 CubicBSplineSegment(s16 *b, u16 t);
inline u16 CubicBSpline(s16 *b, u16 t);
#if (GAME == GAME_SA3)
inline s16 CubicBSplineDirect(s16 *b, u16 t);
#endif

UNUSED s32 BezierCubicInterpolate(s32 p0, s32 p1, s32 p2, s32 p3, s32 t, u8 n);
UNUSED s32 HermiteCubicInterpolate(s32 p0, s32 m0, s32 m1, s32 p1, s32 t, u8 n);

UNUSED inline void Transform2D_Init(Transform2D *transform);
UNUSED void Transform2D_Apply(Transform2D *local, Transform2D *parent, Transform2D *out);
static void Transform2D_ApplyToLocal(Transform2D *local, Transform2D *parent);
UNUSED void Transform2D_ApplyToParent(Transform2D *local, Transform2D *parent);
UNUSED void Transform2D_Invert(Transform2D *transform);

static inline void Vec3_Sub(Vec3_32 *u, Vec3_32 *v, Vec3_16 *out)
{
    out->x = u->x - v->x;
    out->y = u->y - v->y;
    out->z = u->z - v->z;
}

inline void Vec3_Normalize(Vec3_16 *v, u8 n);
UNUSED inline s32 Vec3_Dot(Vec3_16 *u, Vec3_16 *v, u8 n);
inline void Vec3_Cross(Vec3_16 *u, Vec3_16 *v, Vec3_16 *out, u8 n);

UNUSED inline void Quaternion_Init(Quaternion *quat);
UNUSED void Quaternion_Normalize(Quaternion *quat);
UNUSED void Quaternion_Apply(Quaternion *local, Quaternion *parent, Quaternion *out);
UNUSED void Quaternion_ApplyToLocal(Quaternion *local, Quaternion *parent);
UNUSED void Quaternion_ApplyToParent(Quaternion *local, Quaternion *parent);
UNUSED inline void Quaternion_Conjugate(Quaternion *quat);
UNUSED void Quaternion_Invert(Quaternion *quat);
UNUSED inline void Quaternion_RotateX(Quaternion *quat, u16 angle);
UNUSED inline void Quaternion_RotateY(Quaternion *quat, u16 angle);
UNUSED inline void Quaternion_RotateZ(Quaternion *quat, u16 angle);
UNUSED inline void Quaternion_RotateAxis(Quaternion *quat, u16 angle, Vec3_16 *axis);

inline void Transform3D_Clear(Transform3D *transform);
inline void Transform3D_Init(Transform3D *transform);
UNUSED inline void Transform3D_InitTranslation(Transform3D *transform, s32 x, s32 y, s32 z);
UNUSED inline void Transform3D_InitIdentity(Transform3D *transform, s32 m00, s32 m11, s32 m22);
static void Transform3D_Apply(Transform3D *local, Transform3D *parent, Transform3D *out);
UNUSED void Transform3D_ApplyToLocal(Transform3D *local, Transform3D *parent);
UNUSED void Transform3D_ApplyToParent(Transform3D *local, Transform3D *parent);
UNUSED void Transform3D_PointTransform(Vec3_32 *v, Transform3D *transform, Vec3_32 *out);
UNUSED void Transform3D_PointTransformIP(Vec3_32 *v, Transform3D *transform);
UNUSED void Transform3D_PointProject(Transform3D *transform, Vec3_32 *v, Vec3_32 *out);
UNUSED void Transform3D_PointProjectIP(Transform3D *transform, Vec3_32 *v);
UNUSED void Transform3D_RotateX(Transform3D *transform, u16 angle);
UNUSED void Transform3D_RotateY(Transform3D *transform, u16 angle);
UNUSED void Transform3D_RotateZ(Transform3D *transform, u16 angle);
UNUSED void Transform3D_RotateAxis(Transform3D *transform, u16 angle, Vec3_16 *axis);
static void Transform3D_RotateXYZ(Transform3D *transform, u16 angleX, u16 angleY, u16 angleZ);
UNUSED void Transform3D_LookAt(Transform3D *transform, Vec3_32 *pos, Vec3_32 *target, Vec3_16 *originalY);
static void Transform3D_Transpose(Transform3D *transform);
UNUSED void Transform3D_Invert(Transform3D *transform);
UNUSED void Transform3D_InvertRigid(Transform3D *transform);
UNUSED void Transform3D_FromQuaternion(Transform3D *transform, Quaternion *quat);

UNUSED inline void TransformNode_Init(TransformNode *node);
UNUSED inline TransformNode *TransformNode_GetRoot(TransformNode *node);
inline void TransformNode_DetachFromParent(TransformNode *node);
UNUSED inline void TransformNode_AddChild(TransformNode *parent, TransformNode *child);
UNUSED inline u16 TransformNode_GetChildCount(TransformNode *node);
UNUSED inline TransformNode *TransformNode_GetChildByIdx(TransformNode *node, u16 idx);
inline void TransformNode_GetLocal2D(TransformNode *node, Transform2D *transform);
UNUSED inline void TransformNode_SetLocal2D(TransformNode *node, Transform2D *transform);
UNUSED inline void TransformNode_GetWorld2D(TransformNode *node, Transform2D *transform);
UNUSED inline void TransformNode_InitLocal3D(TransformNode *node);
UNUSED inline void TransformNode_InitTRS(TransformNode *node);
static Transform3D *TransformNode_GetLocal3D(TransformNode *node);
inline Transform3D *TransformNode_GetWorld3D(TransformNode *node);
UNUSED inline Transform3D *TransformNode_UpdateWorld3D(TransformNode *node);

UNUSED inline void Math_DummyFunc(void);

#endif // GUARD_MATH_H