// SPDX-License-Identifier: MIT
// Sable Vox3D Physics Engine Integration - JNI Native Bridge

#include <jni.h>
#include <box3d/box3d.h>
#include <box3d/math_functions.h>
#include <box3d/collision.h>
#include <box3d/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <limits.h>

#ifdef _WIN32
#include <windows.h>
typedef CRITICAL_SECTION vox_mutex_t;
#define VOX_MUTEX_INIT(m) InitializeCriticalSection(&(m))
#define VOX_MUTEX_LOCK(m) EnterCriticalSection(&(m))
#define VOX_MUTEX_UNLOCK(m) LeaveCriticalSection(&(m))
#define VOX_MUTEX_DESTROY(m) DeleteCriticalSection(&(m))
#else
#include <pthread.h>
typedef pthread_mutex_t vox_mutex_t;
#define VOX_MUTEX_INIT(m) pthread_mutex_init(&(m), NULL)
#define VOX_MUTEX_LOCK(m) pthread_mutex_lock(&(m))
#define VOX_MUTEX_UNLOCK(m) pthread_mutex_unlock(&(m))
#define VOX_MUTEX_DESTROY(m) pthread_mutex_destroy(&(m))
#endif

#define MAX_VOXEL_COLLIDERS 8192
#define MAX_COLLISIONS 16384
#define CHUNK_HASH_SIZE 65536

typedef struct {
    double min[3];
    double max[3];
} VoxelBox;

typedef struct {
    int id;
    float friction;
    float volume;
    float restitution;
    bool isFluid;
    jobject callback;
    VoxelBox* boxes;
    int boxCount;
    int boxCapacity;
} VoxelColliderDef;

typedef struct {
    int id;
    b3BodyId bodyId;
    double centerOfMass[3];
    b3MassData massData;
    bool hasMassData;
    int minBounds[3];
    int maxBounds[3];
    b3ShapeId* shapes;
    int shapeCount;
    int shapeCapacity;
    b3VoxelData* voxelData;
    bool isVoxelSubLevel;
    bool valid;
} SubLevelBody;

typedef struct {
    b3ShapeId shapeId;
    int contraptionId;
    double sourceCenter[3];
    float halfExtents[3];
    int blockPosition[3];
    int colliderId;
} KinematicContraptionShape;

typedef struct {
    int id;
    int mountId;
    b3BodyId bodyId;
    bool ownsBody;
    double centerOfMass[3];
    // For a mounted contraption this is the collider pose relative to the
    // parent sub-level rigid body, matching Rapier's position_wrt_parent. Keep
    // the translation in double precision until individual hulls are rebuilt.
    double posePosition[3];
    b3Quat poseRotation;
    // Sable reports these in the contraption's local frame. Mounted
    // contraptions have no solver body of their own, so these are also used to
    // approximate their moving surface velocity on each block hull.
    double linearVelocity[3];
    double angularVelocity[3];
    KinematicContraptionShape* shapes;
    int shapeCount;
    int shapeCapacity;
    bool valid;
} KinematicContraption;

typedef struct {
    int64_t handle;
    int idA;
    int idB;
    b3JointId jointId;
    b3JointId filterJointId;
    b3BodyId worldAnchorBodyA;
    b3BodyId worldAnchorBodyB;
    int type; // 0=fixed, 1=rotary, 2=free, 3=generic
    int jointKind; // 0=none, 1=weld, 2=spherical, 3=revolute, 4=prismatic
    bool contactsEnabled;
    // Sable supplies sub-level anchors in plot/block coordinates. Keep those
    // coordinates as doubles and only subtract the current COM immediately
    // before handing the local frame to Box3D. Plot coordinates are too large
    // to survive an intermediate float conversion.
    double sourcePos1[3];
    double sourcePos2[3];
    b3Vec3 pos1, pos2;
    b3Vec3 normal1, normal2;
    b3Quat rot1, rot2;
    double motorTarget[6];
    double motorStiffness[6];
    double motorDamping[6];
    double motorMaxForce[6];
    bool motorHasForceLimit[6];
    bool motorConfigured[6];
    int lockedAxesMask;
    float impulses[6];
    bool valid;
} Vox3DConstraint;

typedef struct {
    int64_t ropeId;
    double pointRadius;
    double firstJointLength;
    int pointCount;
    int pointCapacity;
    b3Pos* points;
    b3BodyId* segmentBodies;
    b3JointId* joints;
    b3JointId startJoint;
    b3JointId endJoint;
    b3BodyId startWorldAnchorBody;
    b3BodyId endWorldAnchorBody;
    bool hasStartAttachment;
    bool hasEndAttachment;
    int startSubLevelId;
    int endSubLevelId;
    double startLocation[3];
    double endLocation[3];
    bool valid;
} Vox3DRope;

typedef struct {
    int idA;
    int idB;
    double forceAmount;
    double normalA[3];
    double normalB[3];
    double pointA[3];
    double pointB[3];
} ReportedCollision;

typedef struct SceneChunkNode {
    int cx, cy, cz;
    int data[4096];
    bool isGlobal;
    int subLevelId;
    b3BodyId staticBodyId;
    b3VoxelData** voxelData;
    int voxelDataCount;
    int voxelDataCapacity;
    struct SceneChunkNode* next;
} SceneChunkNode;

typedef struct {
    b3WorldId worldId;
    b3BodyId groundBodyId;
    double gravity[3];
    double universalDrag;
    int solverSubSteps;
    double lastTimeStep;
    float contactSpeed;
    int minIslandSize;

    SubLevelBody* subLevels;
    int subLevelCount;
    int subLevelCapacity;

    KinematicContraption* contraptions;
    int contraptionCount;
    int contraptionCapacity;

    Vox3DConstraint* constraints;
    int constraintCount;
    int constraintCapacity;

    Vox3DRope* ropes;
    int ropeCount;
    int ropeCapacity;

    SceneChunkNode* chunkBuckets[CHUNK_HASH_SIZE];

    ReportedCollision collisions[MAX_COLLISIONS];
    int collisionCount;

    b3Vec3 wind;
    float aeroDrag;
    float aeroLift;
    float maxAeroSpeed;

    vox_mutex_t mutex;
} Vox3DScene;

// Global state
static VoxelColliderDef g_voxelColliders[MAX_VOXEL_COLLIDERS];
static int g_voxelColliderCount = 0;
static vox_mutex_t g_globalMutex;
static bool g_globalMutexInit = false;
static int g_nextBodyId = 0;
static JavaVM* g_javaVm = NULL;

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void* reserved) {
    (void)reserved;
    g_javaVm = vm;
    return JNI_VERSION_1_8;
}

JNIEXPORT void JNICALL JNI_OnUnload(JavaVM* vm, void* reserved) {
    (void)reserved;
    JNIEnv* env = NULL;
    if ((*vm)->GetEnv(vm, (void**)&env, JNI_VERSION_1_8) == JNI_OK) {
        for (int i = 0; i < g_voxelColliderCount; i++) {
            if (g_voxelColliders[i].callback) {
                (*env)->DeleteGlobalRef(env, g_voxelColliders[i].callback);
                g_voxelColliders[i].callback = NULL;
            }
            free(g_voxelColliders[i].boxes);
            g_voxelColliders[i].boxes = NULL;
            g_voxelColliders[i].boxCount = 0;
            g_voxelColliders[i].boxCapacity = 0;
        }
    }
    g_javaVm = NULL;
}

static void ensure_global_mutex(void) {
    if (!g_globalMutexInit) {
        VOX_MUTEX_INIT(g_globalMutex);
        g_globalMutexInit = true;
    }
}

static b3Vec3 get_local_anchor(Vox3DScene* scene, int bodyIdInt, double x, double y, double z);
static void update_joint_frames(Vox3DScene* scene, Vox3DConstraint* constraint, bool wakeBodies);
static void sync_constraint_contact_filter(Vox3DScene* scene, Vox3DConstraint* constraint);
static bool constraint_backend_valid(Vox3DScene* scene, Vox3DConstraint* constraint);
static void destroy_constraint_resources(Vox3DConstraint* constraint, bool wakeBodies);
static void invalidate_body_dependents(Vox3DScene* scene, int bodyId);
static void apply_constraint_pd_motors(Vox3DScene* scene, Vox3DConstraint* constraint, double timeStep, int motorAxesMask);
static void apply_rotary_constraint_motor(Vox3DScene* scene, Vox3DConstraint* constraint, double timeStep);
static KinematicContraption* find_contraption(Vox3DScene* scene, int id);
static KinematicContraptionShape* get_contraption_shape_data(b3ShapeId shapeId);
static VoxelColliderDef* lookup_contraption_collision_block(
    Vox3DScene* scene, KinematicContraptionShape* shape, b3Pos point,
    int* blockX, int* blockY, int* blockZ, b3Pos* impactPosition);
static b3Vec3 get_contraption_fake_world_velocity(
    Vox3DScene* scene, KinematicContraptionShape* shape, b3Pos point);
static b3BodyId get_rope_mount_body(Vox3DScene* scene, const Vox3DRope* rope, bool end);
static b3Vec3 get_rope_mount_anchor(Vox3DScene* scene, const Vox3DRope* rope, bool end);
static void destroy_rope_world_anchor(Vox3DRope* rope, bool end);

static inline b3Quat make_safe_quat(double x, double y, double z, double w) {
    double lenSq = x*x + y*y + z*z + w*w;
    if (lenSq < 1e-8 || !isfinite(lenSq)) {
        return b3Quat_identity;
    }
    double invLen = 1.0 / sqrt(lenSq);
    return (b3Quat){ { (float)(x * invLen), (float)(y * invLen), (float)(z * invLen) }, (float)(w * invLen) };
}

static uint32_t chunk_hash(int cx, int cy, int cz) {
    uint32_t h = (((uint32_t)cx * 73856093u) ^ ((uint32_t)cy * 19349663u) ^ ((uint32_t)cz * 83492791u));
    return h & (CHUNK_HASH_SIZE - 1);
}

static SceneChunkNode* find_chunk_node(Vox3DScene* scene, int cx, int cy, int cz) {
    uint32_t bucket = chunk_hash(cx, cy, cz);
    SceneChunkNode* cur = scene->chunkBuckets[bucket];
    while (cur) {
        if (cur->cx == cx && cur->cy == cy && cur->cz == cz) return cur;
        cur = cur->next;
    }
    return NULL;
}

static SceneChunkNode* get_or_create_chunk_node(Vox3DScene* scene, int cx, int cy, int cz) {
    uint32_t bucket = chunk_hash(cx, cy, cz);
    SceneChunkNode* cur = scene->chunkBuckets[bucket];
    while (cur) {
        if (cur->cx == cx && cur->cy == cy && cur->cz == cz) return cur;
        cur = cur->next;
    }
    SceneChunkNode* node = (SceneChunkNode*)calloc(1, sizeof(SceneChunkNode));
    if (!node) return NULL;
    node->cx = cx;
    node->cy = cy;
    node->cz = cz;
    node->next = scene->chunkBuckets[bucket];
    scene->chunkBuckets[bucket] = node;
    return node;
}

static void clear_chunk_voxel_data(SceneChunkNode* node) {
    for (int i = 0; i < node->voxelDataCount; i++) {
        if (node->voxelData[i]) b3DestroyVoxelData(node->voxelData[i]);
    }
    free(node->voxelData);
    node->voxelData = NULL;
    node->voxelDataCount = 0;
    node->voxelDataCapacity = 0;
}

static bool add_chunk_voxel_data(SceneChunkNode* node, b3VoxelData* data) {
    if (node->voxelDataCount == node->voxelDataCapacity) {
        int newCapacity = node->voxelDataCapacity == 0 ? 8 : node->voxelDataCapacity * 2;
        b3VoxelData** newData = (b3VoxelData**)realloc(
            node->voxelData, newCapacity * sizeof(b3VoxelData*));
        if (!newData) return false;
        node->voxelData = newData;
        node->voxelDataCapacity = newCapacity;
    }
    node->voxelData[node->voxelDataCount++] = data;
    return true;
}

static void remove_chunk_node(Vox3DScene* scene, int cx, int cy, int cz) {
    uint32_t bucket = chunk_hash(cx, cy, cz);
    SceneChunkNode** prev = &scene->chunkBuckets[bucket];
    SceneChunkNode* cur = *prev;
    while (cur) {
        if (cur->cx == cx && cur->cy == cy && cur->cz == cz) {
            *prev = cur->next;
            clear_chunk_voxel_data(cur);
            if (b3Body_IsValid(cur->staticBodyId)) {
                b3DestroyBody(cur->staticBodyId);
            }
            free(cur);
            return;
        }
        prev = &cur->next;
        cur = cur->next;
    }
}

static void clear_all_chunk_nodes(Vox3DScene* scene) {
    for (int b = 0; b < CHUNK_HASH_SIZE; b++) {
        SceneChunkNode* cur = scene->chunkBuckets[b];
        while (cur) {
            SceneChunkNode* next = cur->next;
            clear_chunk_voxel_data(cur);
            if (b3Body_IsValid(cur->staticBodyId)) {
                b3DestroyBody(cur->staticBodyId);
            }
            free(cur);
            cur = next;
        }
        scene->chunkBuckets[b] = NULL;
    }
}

static SubLevelBody* find_sublevel(Vox3DScene* scene, int id) {
    for (int i = 0; i < scene->subLevelCount; i++) {
        if (scene->subLevels[i].valid && scene->subLevels[i].id == id) {
            return &scene->subLevels[i];
        }
    }
    return NULL;
}

static SubLevelBody* find_sublevel_by_body(Vox3DScene* scene, b3BodyId bodyId) {
    for (int i = 0; i < scene->subLevelCount; i++) {
        if (scene->subLevels[i].valid && B3_ID_EQUALS(scene->subLevels[i].bodyId, bodyId)) {
            return &scene->subLevels[i];
        }
    }
    return NULL;
}

/*
 * Approximate Sable's block-fluid buoyancy contract with one sample at the
 * center of each dynamic block. This preserves configured block volume and
 * fluid classification even though Box3D does not expose Rapier-style
 * intersection pairs for non-colliding fluid voxels.
 */
static void apply_buoyancy_forces(Vox3DScene* scene) {
    for (int s = 0; s < scene->subLevelCount; s++) {
        SubLevelBody* sub = &scene->subLevels[s];
        if (!sub->valid || !sub->isVoxelSubLevel || !b3Body_IsValid(sub->bodyId)) continue;

        b3WorldTransform bodyTransform = b3Body_GetTransform(sub->bodyId);

        for (int bucket = 0; bucket < CHUNK_HASH_SIZE; bucket++) {
            for (SceneChunkNode* node = scene->chunkBuckets[bucket]; node; node = node->next) {
                if (node->isGlobal || node->subLevelId != sub->id) continue;

                for (int bx = 0; bx < 16; bx++) {
                    for (int bz = 0; bz < 16; bz++) {
                        for (int by = 0; by < 16; by++) {
                            int packed = node->data[bx + (bz << 4) + (by << 8)];
                            int colliderId = (packed >> 16) & 0xFFFF;
                            if (colliderId <= 0 || colliderId > g_voxelColliderCount) continue;

                            VoxelColliderDef* solid = &g_voxelColliders[colliderId - 1];
                            if (solid->isFluid || solid->volume <= 0.0f) continue;

                            int blockX = bx + (node->cx << 4);
                            int blockY = by + (node->cy << 4);
                            int blockZ = bz + (node->cz << 4);
                            b3Vec3 localCenter = {
                                (float)((double)blockX + 0.5 - sub->centerOfMass[0]),
                                (float)((double)blockY + 0.5 - sub->centerOfMass[1]),
                                (float)((double)blockZ + 0.5 - sub->centerOfMass[2])
                            };
                            b3Pos worldCenter = b3TransformWorldPoint(bodyTransform, localCenter);

                            int worldX = (int)floor(worldCenter.x);
                            int worldY = (int)floor(worldCenter.y);
                            int worldZ = (int)floor(worldCenter.z);
                            SceneChunkNode* fluidNode = find_chunk_node(scene, worldX >> 4, worldY >> 4, worldZ >> 4);
                            if (!fluidNode || !fluidNode->isGlobal) continue;

                            int fluidIndex = (worldX & 15) + ((worldZ & 15) << 4) + ((worldY & 15) << 8);
                            int fluidPacked = fluidNode->data[fluidIndex];
                            int fluidColliderId = (fluidPacked >> 16) & 0xFFFF;
                            if (fluidColliderId <= 0 || fluidColliderId > g_voxelColliderCount) continue;
                            if (!g_voxelColliders[fluidColliderId - 1].isFluid) continue;

                            b3Vec3 pointVelocity = b3Body_GetWorldPointVelocity(sub->bodyId, worldCenter);
                            float volume = solid->volume;
                            b3Vec3 force = {
                                -pointVelocity.x * 1.7f * volume,
                                (10.5f - pointVelocity.y * 1.7f) * volume,
                                -pointVelocity.z * 1.7f * volume
                            };
                            b3Body_ApplyForce(sub->bodyId, force, worldCenter, true);
                        }
                    }
                }
            }
        }
    }
}

static VoxelColliderDef* lookup_collision_block(
    Vox3DScene* scene, SubLevelBody* sub, b3BodyId sourceBody,
    b3Pos point, b3Vec3 inward,
    int* blockX, int* blockY, int* blockZ, b3Pos* impactPosition)
{
    b3Pos sample = {
        point.x + (double)inward.x * 0.025,
        point.y + (double)inward.y * 0.025,
        point.z + (double)inward.z * 0.025
    };

    if (sub && sub->isVoxelSubLevel) {
        b3WorldTransform transform = b3Body_GetTransform(sub->bodyId);
        b3Vec3 localSample = b3InvTransformWorldPoint(transform, sample);
        b3Vec3 localImpact = b3InvTransformWorldPoint(transform, point);
        *blockX = (int)floor((double)localSample.x + sub->centerOfMass[0]);
        *blockY = (int)floor((double)localSample.y + sub->centerOfMass[1]);
        *blockZ = (int)floor((double)localSample.z + sub->centerOfMass[2]);
        *impactPosition = (b3Pos){
            (double)localImpact.x + sub->centerOfMass[0],
            (double)localImpact.y + sub->centerOfMass[1],
            (double)localImpact.z + sub->centerOfMass[2]
        };
    } else {
        *blockX = (int)floor(sample.x);
        *blockY = (int)floor(sample.y);
        *blockZ = (int)floor(sample.z);
        *impactPosition = point;
    }

    SceneChunkNode* node = find_chunk_node(scene, *blockX >> 4, *blockY >> 4, *blockZ >> 4);
    if (!node) return NULL;
    if (sub && sub->isVoxelSubLevel) {
        if (node->isGlobal || node->subLevelId != sub->id) return NULL;
    } else {
        // An unregistered dynamic shape (BoxHandle, rope segment, etc.) may
        // overlap a terrain voxel at the same contact point. Only attribute
        // this side to global terrain when its actual body is that chunk's
        // static body; otherwise the terrain callback would fire twice.
        if (!node->isGlobal || !b3Body_IsValid(node->staticBodyId) ||
            !B3_ID_EQUALS(node->staticBodyId, sourceBody)) return NULL;
    }

    int index = (*blockX & 15) + ((*blockZ & 15) << 4) + ((*blockY & 15) << 8);
    int colliderId = (node->data[index] >> 16) & 0xFFFF;
    if (colliderId <= 0 || colliderId > g_voxelColliderCount) return NULL;
    return &g_voxelColliders[colliderId - 1];
}

static bool invoke_block_callback(
    JNIEnv* env, VoxelColliderDef* def,
    int x, int y, int z, int otherX, int otherY, int otherZ,
    b3Pos impactPosition, double impactVelocity, bool hasOtherBlock)
{
    if (!def || !def->callback) return false;

    jclass callbackClass = (*env)->GetObjectClass(env, def->callback);
    if (!callbackClass) {
        (*env)->ExceptionClear(env);
        return false;
    }
    jmethodID method = (*env)->GetMethodID(env, callbackClass, "onCollision", "(IIIIIIDDDDZ)[D");
    if (!method) {
        (*env)->ExceptionClear(env);
        (*env)->DeleteLocalRef(env, callbackClass);
        return false;
    }

    jdoubleArray result = (jdoubleArray)(*env)->CallObjectMethod(
        env, def->callback, method,
        (jint)x, (jint)y, (jint)z,
        (jint)otherX, (jint)otherY, (jint)otherZ,
        (jdouble)impactPosition.x, (jdouble)impactPosition.y, (jdouble)impactPosition.z,
        (jdouble)impactVelocity, (jboolean)(hasOtherBlock ? JNI_TRUE : JNI_FALSE));

    bool removeCollision = false;
    if ((*env)->ExceptionCheck(env)) {
        (*env)->ExceptionClear(env);
    } else if (result && (*env)->GetArrayLength(env, result) >= 4) {
        jdouble values[4];
        (*env)->GetDoubleArrayRegion(env, result, 0, 4, values);
        // Box3D's pre-solve hook can keep or remove a contact, but it has no
        // equivalent for Sable/Rapier's per-contact tangent velocity result.
        removeCollision = values[3] != 0.0;
    }

    if (result) (*env)->DeleteLocalRef(env, result);
    (*env)->DeleteLocalRef(env, callbackClass);
    return removeCollision;
}

static bool vox_pre_solve_callback(
    b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3Pos point, b3Vec3 normal, void* context)
{
    Vox3DScene* scene = (Vox3DScene*)context;
    if (!scene || !g_javaVm) return true;

    b3BodyId bodyA = b3Shape_GetBody(shapeIdA);
    b3BodyId bodyB = b3Shape_GetBody(shapeIdB);
    SubLevelBody* subA = find_sublevel_by_body(scene, bodyA);
    SubLevelBody* subB = find_sublevel_by_body(scene, bodyB);
    KinematicContraptionShape* contraptionShapeA = get_contraption_shape_data(shapeIdA);
    KinematicContraptionShape* contraptionShapeB = get_contraption_shape_data(shapeIdB);

    int ax = 0, ay = 0, az = 0, bx = 0, by = 0, bz = 0;
    b3Pos impactA = { 0.0, 0.0, 0.0 };
    b3Pos impactB = { 0.0, 0.0, 0.0 };
    VoxelColliderDef* defA = contraptionShapeA
        ? lookup_contraption_collision_block(
            scene, contraptionShapeA, point, &ax, &ay, &az, &impactA)
        : lookup_collision_block(
            scene, subA, bodyA, point, b3Neg(normal), &ax, &ay, &az, &impactA);
    VoxelColliderDef* defB = contraptionShapeB
        ? lookup_contraption_collision_block(
            scene, contraptionShapeB, point, &bx, &by, &bz, &impactB)
        : lookup_collision_block(
            scene, subB, bodyB, point, normal, &bx, &by, &bz, &impactB);
    if ((!defA || !defA->callback) && (!defB || !defB->callback)) return true;

    b3Vec3 velocityA = b3Add(
        b3Body_GetWorldPointVelocity(bodyA, point),
        get_contraption_fake_world_velocity(scene, contraptionShapeA, point));
    b3Vec3 velocityB = b3Add(
        b3Body_GetWorldPointVelocity(bodyB, point),
        get_contraption_fake_world_velocity(scene, contraptionShapeB, point));
    double impactVelocity = fmax(0.0, (double)b3Dot(b3Sub(velocityA, velocityB), normal));

    JNIEnv* env = NULL;
    bool attached = false;
    jint status = (*g_javaVm)->GetEnv(g_javaVm, (void**)&env, JNI_VERSION_1_8);
    if (status == JNI_EDETACHED) {
        if ((*g_javaVm)->AttachCurrentThread(g_javaVm, (void**)&env, NULL) != JNI_OK) return true;
        attached = true;
    } else if (status != JNI_OK) {
        return true;
    }

    bool removeCollision = invoke_block_callback(
        env, defA, ax, ay, az, bx, by, bz, impactA, impactVelocity,
        contraptionShapeB != NULL || (subB != NULL && subB->isVoxelSubLevel));
    removeCollision |= invoke_block_callback(
        env, defB, bx, by, bz, ax, ay, az, impactB, impactVelocity,
        contraptionShapeA != NULL || (subA != NULL && subA->isVoxelSubLevel));

    if (attached) (*g_javaVm)->DetachCurrentThread(g_javaVm);
    return !removeCollision;
}

static SubLevelBody* alloc_sublevel(Vox3DScene* scene, int id) {
    SubLevelBody* sub = find_sublevel(scene, id);
    if (sub) return sub;

    for (int i = 0; i < scene->subLevelCount; i++) {
        if (!scene->subLevels[i].valid) {
            scene->subLevels[i].id = id;
            scene->subLevels[i].valid = true;
            scene->subLevels[i].shapeCount = 0;
            scene->subLevels[i].shapeCapacity = 0;
            scene->subLevels[i].shapes = NULL;
            scene->subLevels[i].voxelData = NULL;
            scene->subLevels[i].isVoxelSubLevel = false;
            scene->subLevels[i].minBounds[0] = 1;
            scene->subLevels[i].maxBounds[0] = 0;
            return &scene->subLevels[i];
        }
    }

    if (scene->subLevelCount >= scene->subLevelCapacity) {
        int newCap = scene->subLevelCapacity == 0 ? 1024 : scene->subLevelCapacity * 2;
        SubLevelBody* newArr = (SubLevelBody*)realloc(scene->subLevels, newCap * sizeof(SubLevelBody));
        if (!newArr) return NULL;
        for (int i = scene->subLevelCapacity; i < newCap; i++) {
            memset(&newArr[i], 0, sizeof(SubLevelBody));
        }
        scene->subLevels = newArr;
        scene->subLevelCapacity = newCap;
    }

    int idx = scene->subLevelCount++;
    scene->subLevels[idx].id = id;
    scene->subLevels[idx].valid = true;
    scene->subLevels[idx].shapeCount = 0;
    scene->subLevels[idx].shapeCapacity = 0;
    scene->subLevels[idx].shapes = NULL;
    scene->subLevels[idx].voxelData = NULL;
    scene->subLevels[idx].isVoxelSubLevel = false;
    scene->subLevels[idx].minBounds[0] = 1;
    scene->subLevels[idx].maxBounds[0] = 0;
    return &scene->subLevels[idx];
}

static KinematicContraption* find_contraption(Vox3DScene* scene, int id) {
    for (int i = 0; i < scene->contraptionCount; i++) {
        if (scene->contraptions[i].valid && scene->contraptions[i].id == id) {
            return &scene->contraptions[i];
        }
    }
    return NULL;
}

static KinematicContraption* alloc_contraption(Vox3DScene* scene, int id) {
    KinematicContraption* con = find_contraption(scene, id);
    if (con) return con;

    for (int i = 0; i < scene->contraptionCount; i++) {
        if (!scene->contraptions[i].valid) {
            memset(&scene->contraptions[i], 0, sizeof(KinematicContraption));
            scene->contraptions[i].id = id;
            scene->contraptions[i].valid = true;
            scene->contraptions[i].mountId = -1;
            scene->contraptions[i].bodyId = b3_nullBodyId;
            scene->contraptions[i].poseRotation = b3Quat_identity;
            return &scene->contraptions[i];
        }
    }

    if (scene->contraptionCount >= scene->contraptionCapacity) {
        int newCap = scene->contraptionCapacity == 0 ? 512 : scene->contraptionCapacity * 2;
        KinematicContraption* newArr = (KinematicContraption*)realloc(scene->contraptions, newCap * sizeof(KinematicContraption));
        if (!newArr) return NULL;
        for (int i = scene->contraptionCapacity; i < newCap; i++) {
            memset(&newArr[i], 0, sizeof(KinematicContraption));
        }
        scene->contraptions = newArr;
        scene->contraptionCapacity = newCap;
    }

    int idx = scene->contraptionCount++;
    memset(&scene->contraptions[idx], 0, sizeof(KinematicContraption));
    scene->contraptions[idx].id = id;
    scene->contraptions[idx].valid = true;
    scene->contraptions[idx].mountId = -1;
    scene->contraptions[idx].bodyId = b3_nullBodyId;
    scene->contraptions[idx].poseRotation = b3Quat_identity;
    return &scene->contraptions[idx];
}

static bool ensure_contraption_shape_capacity(KinematicContraption* con, int requiredCapacity) {
    if (!con || requiredCapacity < 0) return false;
    if (requiredCapacity <= con->shapeCapacity) return true;
    if ((size_t)requiredCapacity > SIZE_MAX / sizeof(KinematicContraptionShape)) return false;

    int newCapacity = con->shapeCapacity > 0 ? con->shapeCapacity : 64;
    while (newCapacity < requiredCapacity) {
        if (newCapacity > INT_MAX / 2) {
            newCapacity = requiredCapacity;
            break;
        }
        newCapacity *= 2;
    }

    KinematicContraptionShape* newShapes = (KinematicContraptionShape*)realloc(
        con->shapes, (size_t)newCapacity * sizeof(KinematicContraptionShape));
    if (!newShapes) return false;

    con->shapes = newShapes;
    con->shapeCapacity = newCapacity;

    // Shape user data points directly at these records. Repair the pointers if
    // realloc moved the descriptor array.
    for (int i = 0; i < con->shapeCount; i++) {
        if (b3Shape_IsValid(con->shapes[i].shapeId)) {
            b3Shape_SetUserData(con->shapes[i].shapeId, &con->shapes[i]);
        }
    }
    return true;
}

static b3Vec3 get_contraption_shape_velocity(const KinematicContraption* con,
                                              const KinematicContraptionShape* shape) {
    b3Vec3 localCenter = {
        (float)(shape->sourceCenter[0] - con->centerOfMass[0]),
        (float)(shape->sourceCenter[1] - con->centerOfMass[1]),
        (float)(shape->sourceCenter[2] - con->centerOfMass[2])
    };
    b3Vec3 localLinear = {
        (float)con->linearVelocity[0],
        (float)con->linearVelocity[1],
        (float)con->linearVelocity[2]
    };
    b3Vec3 localAngular = {
        (float)con->angularVelocity[0],
        (float)con->angularVelocity[1],
        (float)con->angularVelocity[2]
    };
    b3Vec3 velocity = b3Add(localLinear, b3Cross(localAngular, localCenter));
    return b3RotateVector(con->poseRotation, velocity);
}

static b3Transform get_contraption_shape_transform(const KinematicContraption* con,
                                                    const KinematicContraptionShape* shape) {
    b3Transform transform = b3Transform_identity;
    b3Vec3 localCenter = {
        (float)(shape->sourceCenter[0] - con->centerOfMass[0]),
        (float)(shape->sourceCenter[1] - con->centerOfMass[1]),
        (float)(shape->sourceCenter[2] - con->centerOfMass[2])
    };

    if (con->ownsBody) {
        transform.p = localCenter;
        return transform;
    }

    b3Vec3 rotatedCenter = b3RotateVector(con->poseRotation, localCenter);
    transform.p = (b3Vec3){
        (float)(con->posePosition[0] + (double)rotatedCenter.x),
        (float)(con->posePosition[1] + (double)rotatedCenter.y),
        (float)(con->posePosition[2] + (double)rotatedCenter.z)
    };
    transform.q = con->poseRotation;
    return transform;
}

static b3ShapeId create_contraption_shape(KinematicContraption* con,
                                           KinematicContraptionShape* shape) {
    if (!con || !shape || !b3Body_IsValid(con->bodyId) ||
        shape->colliderId <= 0 || shape->colliderId > g_voxelColliderCount) {
        return b3_nullShapeId;
    }

    VoxelColliderDef* def = &g_voxelColliders[shape->colliderId - 1];
    b3Transform shapeTransform = get_contraption_shape_transform(con, shape);
    b3BoxHull boxHull = b3MakeTransformedBoxHull(
        shape->halfExtents[0], shape->halfExtents[1], shape->halfExtents[2], shapeTransform);

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.userData = shape;
    shapeDef.baseMaterial.friction = def->friction;
    shapeDef.baseMaterial.restitution = def->restitution;
    shapeDef.enableHitEvents = true;
    shapeDef.enablePreSolveEvents = true;
    shapeDef.updateBodyMass = false;
    if (!con->ownsBody) {
        // Box exposes only a constant conveyor/surface velocity per hull. A
        // block-sized sample at the hull center is a close approximation of
        // Rapier's point-dependent fake linear/angular velocity.
        shapeDef.baseMaterial.tangentVelocity = get_contraption_shape_velocity(con, shape);
    }
    return b3CreateHullShape(con->bodyId, &shapeDef, &boxHull.base);
}

static void destroy_contraption_shapes(KinematicContraption* con, bool wakeBody) {
    if (!con) return;
    for (int i = 0; i < con->shapeCount; i++) {
        if (b3Shape_IsValid(con->shapes[i].shapeId)) {
            b3DestroyShape(con->shapes[i].shapeId, false);
        }
        con->shapes[i].shapeId = b3_nullShapeId;
    }
    if (wakeBody && b3Body_IsValid(con->bodyId)) {
        b3Body_SetAwake(con->bodyId, true);
    }
}

static void rebuild_contraption_shapes(KinematicContraption* con) {
    if (!con) return;
    destroy_contraption_shapes(con, false);
    if (!b3Body_IsValid(con->bodyId)) return;
    for (int i = 0; i < con->shapeCount; i++) {
        con->shapes[i].shapeId = create_contraption_shape(con, &con->shapes[i]);
    }
    if (!con->ownsBody) b3Body_SetAwake(con->bodyId, true);
}

static void release_contraption(Vox3DScene* scene, KinematicContraption* con,
                                bool invalidateDependents, bool parentBeingDestroyed) {
    if (!con) return;
    if (invalidateDependents) invalidate_body_dependents(scene, con->id);

    destroy_contraption_shapes(con, !con->ownsBody && !parentBeingDestroyed);
    free(con->shapes);
    con->shapes = NULL;
    con->shapeCount = 0;
    con->shapeCapacity = 0;

    if (con->ownsBody && b3Body_IsValid(con->bodyId)) {
        b3DestroyBody(con->bodyId);
    }
    con->bodyId = b3_nullBodyId;
    con->ownsBody = false;
    con->valid = false;
}

static KinematicContraptionShape* get_contraption_shape_data(b3ShapeId shapeId) {
    if (!b3Shape_IsValid(shapeId)) return NULL;
    return (KinematicContraptionShape*)b3Shape_GetUserData(shapeId);
}

static b3Vec3 get_contraption_local_point(const KinematicContraption* con, b3Pos point) {
    b3WorldTransform bodyTransform = b3Body_GetTransform(con->bodyId);
    b3Vec3 bodyLocal = b3InvTransformWorldPoint(bodyTransform, point);
    if (con->ownsBody) return bodyLocal;

    b3Vec3 posePosition = {
        (float)con->posePosition[0],
        (float)con->posePosition[1],
        (float)con->posePosition[2]
    };
    return b3InvRotateVector(
        con->poseRotation, b3Sub(bodyLocal, posePosition));
}

static b3Pos get_contraption_logical_point(const KinematicContraption* con, b3Pos point) {
    b3Vec3 contraptionLocal = get_contraption_local_point(con, point);
    return (b3Pos){
        (double)contraptionLocal.x + con->centerOfMass[0],
        (double)contraptionLocal.y + con->centerOfMass[1],
        (double)contraptionLocal.z + con->centerOfMass[2]
    };
}

static VoxelColliderDef* lookup_contraption_collision_block(
    Vox3DScene* scene, KinematicContraptionShape* shape, b3Pos point,
    int* blockX, int* blockY, int* blockZ, b3Pos* impactPosition)
{
    if (!shape || shape->colliderId <= 0 || shape->colliderId > g_voxelColliderCount) {
        return NULL;
    }
    KinematicContraption* con = find_contraption(scene, shape->contraptionId);
    if (!con || !b3Body_IsValid(con->bodyId)) return NULL;

    *blockX = shape->blockPosition[0];
    *blockY = shape->blockPosition[1];
    *blockZ = shape->blockPosition[2];
    *impactPosition = get_contraption_logical_point(con, point);
    return &g_voxelColliders[shape->colliderId - 1];
}

static b3Vec3 get_contraption_fake_world_velocity(
    Vox3DScene* scene, KinematicContraptionShape* shape, b3Pos point)
{
    if (!shape) return b3Vec3_zero;
    KinematicContraption* con = find_contraption(scene, shape->contraptionId);
    if (!con || con->ownsBody || !b3Body_IsValid(con->bodyId)) return b3Vec3_zero;

    b3Vec3 localPoint = get_contraption_local_point(con, point);
    b3Vec3 linear = {
        (float)con->linearVelocity[0],
        (float)con->linearVelocity[1],
        (float)con->linearVelocity[2]
    };
    b3Vec3 angular = {
        (float)con->angularVelocity[0],
        (float)con->angularVelocity[1],
        (float)con->angularVelocity[2]
    };
    b3Vec3 localVelocity = b3Add(linear, b3Cross(angular, localPoint));
    b3Vec3 bodyLocalVelocity = b3RotateVector(con->poseRotation, localVelocity);
    b3WorldTransform bodyTransform = b3Body_GetTransform(con->bodyId);
    return b3RotateVector(bodyTransform.q, bodyLocalVelocity);
}

static Vox3DConstraint* find_constraint(Vox3DScene* scene, int64_t handle) {
    for (int i = 0; i < scene->constraintCount; i++) {
        if (scene->constraints[i].valid && scene->constraints[i].handle == handle) {
            return &scene->constraints[i];
        }
    }
    return NULL;
}

static Vox3DConstraint* alloc_constraint(Vox3DScene* scene, int64_t handle) {
    Vox3DConstraint* c = find_constraint(scene, handle);
    if (c) return c;

    for (int i = 0; i < scene->constraintCount; i++) {
        if (!scene->constraints[i].valid) {
            memset(&scene->constraints[i], 0, sizeof(Vox3DConstraint));
            scene->constraints[i].handle = handle;
            scene->constraints[i].valid = true;
            scene->constraints[i].contactsEnabled = false;
            scene->constraints[i].jointId = b3_nullJointId;
            scene->constraints[i].filterJointId = b3_nullJointId;
            scene->constraints[i].worldAnchorBodyA = b3_nullBodyId;
            scene->constraints[i].worldAnchorBodyB = b3_nullBodyId;
            return &scene->constraints[i];
        }
    }

    if (scene->constraintCount >= scene->constraintCapacity) {
        int newCap = scene->constraintCapacity == 0 ? 2048 : scene->constraintCapacity * 2;
        Vox3DConstraint* newArr = (Vox3DConstraint*)realloc(scene->constraints, newCap * sizeof(Vox3DConstraint));
        if (!newArr) return NULL;
        for (int i = scene->constraintCapacity; i < newCap; i++) {
            memset(&newArr[i], 0, sizeof(Vox3DConstraint));
        }
        scene->constraints = newArr;
        scene->constraintCapacity = newCap;
    }

    int idx = scene->constraintCount++;
    memset(&scene->constraints[idx], 0, sizeof(Vox3DConstraint));
    scene->constraints[idx].handle = handle;
    scene->constraints[idx].valid = true;
    scene->constraints[idx].contactsEnabled = false;
    scene->constraints[idx].jointId = b3_nullJointId;
    scene->constraints[idx].filterJointId = b3_nullJointId;
    scene->constraints[idx].worldAnchorBodyA = b3_nullBodyId;
    scene->constraints[idx].worldAnchorBodyB = b3_nullBodyId;
    return &scene->constraints[idx];
}

static Vox3DRope* find_rope(Vox3DScene* scene, int64_t ropeId) {
    for (int i = 0; i < scene->ropeCount; i++) {
        if (scene->ropes[i].valid && scene->ropes[i].ropeId == ropeId) {
            return &scene->ropes[i];
        }
    }
    return NULL;
}

static Vox3DRope* alloc_rope(Vox3DScene* scene, int64_t ropeId) {
    Vox3DRope* r = find_rope(scene, ropeId);
    if (r) return r;

    for (int i = 0; i < scene->ropeCount; i++) {
        if (!scene->ropes[i].valid) {
            memset(&scene->ropes[i], 0, sizeof(Vox3DRope));
            scene->ropes[i].ropeId = ropeId;
            scene->ropes[i].valid = true;
            scene->ropes[i].startSubLevelId = -1;
            scene->ropes[i].endSubLevelId = -1;
            return &scene->ropes[i];
        }
    }

    if (scene->ropeCount >= scene->ropeCapacity) {
        int newCap = scene->ropeCapacity == 0 ? 256 : scene->ropeCapacity * 2;
        Vox3DRope* newArr = (Vox3DRope*)realloc(scene->ropes, newCap * sizeof(Vox3DRope));
        if (!newArr) return NULL;
        for (int i = scene->ropeCapacity; i < newCap; i++) {
            memset(&newArr[i], 0, sizeof(Vox3DRope));
        }
        scene->ropes = newArr;
        scene->ropeCapacity = newCap;
    }

    int idx = scene->ropeCount++;
    memset(&scene->ropes[idx], 0, sizeof(Vox3DRope));
    scene->ropes[idx].ropeId = ropeId;
    scene->ropes[idx].valid = true;
    scene->ropes[idx].startSubLevelId = -1;
    scene->ropes[idx].endSubLevelId = -1;
    return &scene->ropes[idx];
}

static bool ensure_rope_capacity(Vox3DRope* rope, int requiredCapacity) {
    if (requiredCapacity <= rope->pointCapacity) return true;

    int newCapacity = rope->pointCapacity > 0 ? rope->pointCapacity : 8;
    while (newCapacity < requiredCapacity) {
        if (newCapacity > INT_MAX / 2) {
            newCapacity = requiredCapacity;
            break;
        }
        newCapacity *= 2;
    }

    b3Pos* newPoints = (b3Pos*)malloc((size_t)newCapacity * sizeof(b3Pos));
    b3BodyId* newBodies = (b3BodyId*)malloc((size_t)newCapacity * sizeof(b3BodyId));
    b3JointId* newJoints = (b3JointId*)malloc((size_t)newCapacity * sizeof(b3JointId));
    if (!newPoints || !newBodies || !newJoints) {
        free(newPoints);
        free(newBodies);
        free(newJoints);
        return false;
    }

    if (rope->pointCount > 0) {
        memcpy(newPoints, rope->points, (size_t)rope->pointCount * sizeof(b3Pos));
        memcpy(newBodies, rope->segmentBodies, (size_t)rope->pointCount * sizeof(b3BodyId));
        memcpy(newJoints, rope->joints, (size_t)rope->pointCount * sizeof(b3JointId));
    }
    for (int i = rope->pointCount; i < newCapacity; i++) {
        newPoints[i] = (b3Pos){ 0.0, 0.0, 0.0 };
        newBodies[i] = b3_nullBodyId;
        newJoints[i] = b3_nullJointId;
    }

    free(rope->points);
    free(rope->segmentBodies);
    free(rope->joints);
    rope->points = newPoints;
    rope->segmentBodies = newBodies;
    rope->joints = newJoints;
    rope->pointCapacity = newCapacity;
    return true;
}

static b3BodyId get_body_by_id(Vox3DScene* scene, int id) {
    if (id == -1) return scene->groundBodyId;
    SubLevelBody* sub = find_sublevel(scene, id);
    if (sub && sub->valid) return sub->bodyId;
    KinematicContraption* con = find_contraption(scene, id);
    if (con && con->valid) return con->bodyId;
    return b3_nullBodyId;
}

static void invalidate_body_dependents(Vox3DScene* scene, int bodyId) {
    for (int i = 0; i < scene->constraintCount; i++) {
        Vox3DConstraint* constraint = &scene->constraints[i];
        if (!constraint->valid ||
            (constraint->idA != bodyId && constraint->idB != bodyId)) {
            continue;
        }
        destroy_constraint_resources(constraint, true);
        constraint->valid = false;
    }

    for (int i = 0; i < scene->ropeCount; i++) {
        Vox3DRope* rope = &scene->ropes[i];
        if (!rope->valid) continue;

        if (rope->hasStartAttachment && rope->startSubLevelId == bodyId) {
            if (b3Joint_IsValid(rope->startJoint)) b3DestroyJoint(rope->startJoint, true);
            rope->startJoint = b3_nullJointId;
            rope->hasStartAttachment = false;
            rope->startSubLevelId = -1;
        }
        if (rope->hasEndAttachment && rope->endSubLevelId == bodyId) {
            if (b3Joint_IsValid(rope->endJoint)) b3DestroyJoint(rope->endJoint, true);
            rope->endJoint = b3_nullJointId;
            rope->hasEndAttachment = false;
            rope->endSubLevelId = -1;
        }
    }
}

static void release_mounted_contraptions(Vox3DScene* scene, int mountId) {
    for (int i = 0; i < scene->contraptionCount; i++) {
        KinematicContraption* con = &scene->contraptions[i];
        if (!con->valid || con->ownsBody || con->mountId != mountId) continue;
        // These hulls are children of the parent body. Remove them before the
        // parent disappears, but never destroy the shared parent body here.
        release_contraption(scene, con, true, true);
    }
}

static b3BodyId create_world_anchor_body(Vox3DScene* scene, double x, double y, double z) {
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_staticBody;
    bodyDef.position = (b3Pos){ x, y, z };
    return b3CreateBody(scene->worldId, &bodyDef);
}

static void rebuild_sublevel_shapes(Vox3DScene* scene, SubLevelBody* sub) {
    if (!sub || !b3Body_IsValid(sub->bodyId)) return;

    // Destroy existing shapes on body
    for (int i = 0; i < sub->shapeCount; i++) {
        if (b3Shape_IsValid(sub->shapes[i])) {
            b3DestroyShape(sub->shapes[i], false);
        }
    }
    sub->shapeCount = 0;

    if (sub->voxelData) {
        b3DestroyVoxelData(sub->voxelData);
        sub->voxelData = NULL;
    }

    for (int b = 0; b < CHUNK_HASH_SIZE; b++) {
        SceneChunkNode* node = scene->chunkBuckets[b];
        while (node) {
            int cx = node->cx;
            int cy = node->cy;
            int cz = node->cz;

            bool belongs = false;
            if (!node->isGlobal) {
                if (node->subLevelId == sub->id) {
                    belongs = true;
                } else if (node->subLevelId == -1 && sub->minBounds[0] <= sub->maxBounds[0]) {
                    int cMinX = sub->minBounds[0] >> 4;
                    int cMaxX = sub->maxBounds[0] >> 4;
                    int cMinY = sub->minBounds[1] >> 4;
                    int cMaxY = sub->maxBounds[1] >> 4;
                    int cMinZ = sub->minBounds[2] >> 4;
                    int cMaxZ = sub->maxBounds[2] >> 4;
                    if (cx >= cMinX && cx <= cMaxX && cy >= cMinY && cy <= cMaxY && cz >= cMinZ && cz <= cMaxZ) {
                        node->subLevelId = sub->id;
                        belongs = true;
                    }
                }
            }

            if (belongs) {
                for (int bx = 0; bx < 16; bx++) {
                    for (int bz = 0; bz < 16; bz++) {
                        for (int by = 0; by < 16; by++) {
                            int idx = bx + (bz << 4) + (by << 8);
                            int packed = node->data[idx];
                            int colliderId = (packed >> 16) & 0xFFFF;
                            if (colliderId <= 0 || colliderId > g_voxelColliderCount) continue;

                            VoxelColliderDef* def = &g_voxelColliders[colliderId - 1];
                            if (def->isFluid || def->boxCount == 0) continue;

                            int wx = bx + (cx << 4);
                            int wy = by + (cy << 4);
                            int wz = bz + (cz << 4);

                            double lx = (double)wx - sub->centerOfMass[0];
                            double ly = (double)wy - sub->centerOfMass[1];
                            double lz = (double)wz - sub->centerOfMass[2];

                            for (int bi = 0; bi < def->boxCount; bi++) {
                                VoxelBox* vb = &def->boxes[bi];
                                float hx = (float)((vb->max[0] - vb->min[0]) * 0.5);
                                float hy = (float)((vb->max[1] - vb->min[1]) * 0.5);
                                float hz = (float)((vb->max[2] - vb->min[2]) * 0.5);
                                if (hx <= 0.0001f || hy <= 0.0001f || hz <= 0.0001f) continue;

                                b3Vec3 offset = {
                                    (float)(lx + vb->min[0] + (double)hx),
                                    (float)(ly + vb->min[1] + (double)hy),
                                    (float)(lz + vb->min[2] + (double)hz)
                                };

                                b3BoxHull boxHull = b3MakeOffsetBoxHull(hx, hy, hz, offset);
                                b3ShapeDef sDef = b3DefaultShapeDef();
                                sDef.baseMaterial.friction = def->friction;
                                sDef.baseMaterial.restitution = def->restitution;
                                sDef.enableHitEvents = true;
                                sDef.enablePreSolveEvents = true;
                                sDef.updateBodyMass = false;

                                b3ShapeId sId = b3CreateHullShape(sub->bodyId, &sDef, &boxHull.base);

                                if (sub->shapeCount == sub->shapeCapacity) {
                                    int newCap = sub->shapeCapacity == 0 ? 128 : sub->shapeCapacity * 2;
                                    b3ShapeId* newArr = (b3ShapeId*)realloc(sub->shapes, newCap * sizeof(b3ShapeId));
                                    if (newArr) {
                                        sub->shapes = newArr;
                                        sub->shapeCapacity = newCap;
                                    }
                                }
                                if (sub->shapeCount < sub->shapeCapacity) {
                                    sub->shapes[sub->shapeCount++] = sId;
                                }
                            }
                        }
                    }
                }
            }
            node = node->next;
        }
    }

    if (b3Body_IsValid(sub->bodyId)) {
        if (sub->hasMassData) {
            b3Body_SetMassData(sub->bodyId, sub->massData);
        } else {
            b3Body_ApplyMassFromShapes(sub->bodyId);
        }
    }
}

// -------------------------------------------------------------------------
// JNI Implementation for dev.ryanhcode.sable.physics.impl.vox3d.Vox3D
// -------------------------------------------------------------------------

JNIEXPORT jlong JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_initialize(
    JNIEnv *env, jclass clazz, jdouble gx, jdouble gy, jdouble gz, jdouble universalDrag)
{
    (void)env; (void)clazz;
    ensure_global_mutex();

    Vox3DScene* scene = (Vox3DScene*)calloc(1, sizeof(Vox3DScene));
    if (!scene) return 0;

    VOX_MUTEX_INIT(scene->mutex);
    scene->gravity[0] = gx;
    scene->gravity[1] = gy;
    scene->gravity[2] = gz;
    scene->universalDrag = universalDrag;
    scene->solverSubSteps = 4;
    scene->lastTimeStep = 1.0 / 20.0;
    scene->contactSpeed = 30.0f;
    scene->minIslandSize = 128;

    scene->wind = b3Vec3_zero;
    scene->aeroDrag = 1.0f;
    scene->aeroLift = 1.2f;
    scene->maxAeroSpeed = 50.0f;

    b3WorldDef worldDef = b3DefaultWorldDef();
    b3Vec3 g = { (float)gx, (float)gy, (float)gz };
    worldDef.gravity = g;
    worldDef.enableContinuous = true;
    worldDef.enableSleep = true;
    worldDef.workerCount = 4;
    worldDef.contactHertz = 60.0f;
    worldDef.contactDampingRatio = 2.0f;
    worldDef.contactSpeed = 30.0f;

    scene->worldId = b3CreateWorld(&worldDef);
    b3World_SetPreSolveCallback(scene->worldId, vox_pre_solve_callback, scene);

    // Create ground static body
    b3BodyDef groundDef = b3DefaultBodyDef();
    groundDef.type = b3_staticBody;
    scene->groundBodyId = b3CreateBody(scene->worldId, &groundDef);

    return (jlong)(uintptr_t)scene;
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_dispose(
    JNIEnv *env, jclass clazz, jlong sceneHandle)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);

    clear_all_chunk_nodes(scene);

    for (int i = 0; i < scene->subLevelCount; i++) {
        if (scene->subLevels[i].voxelData) {
            b3DestroyVoxelData(scene->subLevels[i].voxelData);
            scene->subLevels[i].voxelData = NULL;
        }
        if (scene->subLevels[i].shapes) {
            free(scene->subLevels[i].shapes);
            scene->subLevels[i].shapes = NULL;
        }
    }
    if (scene->subLevels) {
        free(scene->subLevels);
        scene->subLevels = NULL;
    }

    for (int i = 0; i < scene->contraptionCount; i++) {
        if (scene->contraptions[i].shapes) {
            free(scene->contraptions[i].shapes);
            scene->contraptions[i].shapes = NULL;
        }
    }
    if (scene->contraptions) {
        free(scene->contraptions);
        scene->contraptions = NULL;
    }

    if (scene->constraints) {
        free(scene->constraints);
        scene->constraints = NULL;
    }

    for (int i = 0; i < scene->ropeCount; i++) {
        free(scene->ropes[i].points);
        free(scene->ropes[i].segmentBodies);
        free(scene->ropes[i].joints);
        scene->ropes[i].points = NULL;
        scene->ropes[i].segmentBodies = NULL;
        scene->ropes[i].joints = NULL;
        scene->ropes[i].pointCapacity = 0;
    }
    if (scene->ropes) {
        free(scene->ropes);
        scene->ropes = NULL;
    }

    if (b3World_IsValid(scene->worldId)) {
        b3DestroyWorld(scene->worldId);
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
    VOX_MUTEX_DESTROY(scene->mutex);
    free(scene);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_tick(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jdouble timeStep)
{
    (void)env; (void)clazz; (void)timeStep;
    if (sceneHandle == 0) return;
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_step(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jdouble timeStep)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    if (isfinite(timeStep) && timeStep > 0.0) {
        scene->lastTimeStep = timeStep;
    }

    // Match Sable's Rapier backend: retain source anchors in double-precision
    // plot coordinates and rebase them against the latest center of mass before
    // every simulation step. Frame setters do not wake sleeping Box3D bodies.
    for (int i = 0; i < scene->constraintCount; i++) {
        Vox3DConstraint* constraint = &scene->constraints[i];
        if (!constraint->valid) continue;
        if (!constraint_backend_valid(scene, constraint)) {
            // Box3D destroys a joint automatically when either connected body
            // disappears. Mirror Rapier's handle lifecycle instead of leaving
            // a valid-looking Sable handle backed by no bodies or joint.
            destroy_constraint_resources(constraint, false);
            constraint->valid = false;
            continue;
        }
        if (!constraint->contactsEnabled &&
            (constraint->type == 2 ||
             (constraint->type == 3 && (constraint->lockedAxesMask & 0x3F) == 0)) &&
            !b3Joint_IsValid(constraint->filterJointId)) {
            sync_constraint_contact_filter(scene, constraint);
        }
        update_joint_frames(scene, constraint, false);
        if (constraint->type == 2) {
            apply_constraint_pd_motors(scene, constraint, timeStep, 0x3F);
        } else if (constraint->type == 3) {
            apply_constraint_pd_motors(scene, constraint, timeStep,
                (~constraint->lockedAxesMask) & 0x3F);
        } else if (constraint->type == 1) {
            apply_rotary_constraint_motor(scene, constraint, timeStep);
        }
    }

    // Update rope attachment local frames if bodies moved/shifted center of mass.
    for (int r = 0; r < scene->ropeCount; r++) {
        if (!scene->ropes[r].valid) continue;
        Vox3DRope* rope = &scene->ropes[r];
        if (rope->hasStartAttachment) {
            if (b3Joint_IsValid(rope->startJoint)) {
                b3Transform xfA, xfB;
                xfA.p = get_rope_mount_anchor(scene, rope, false);
                xfA.q = b3Quat_identity;
                xfB.p = b3Vec3_zero;
                xfB.q = b3Quat_identity;
                b3Joint_SetLocalFrameA(rope->startJoint, xfA);
                b3Joint_SetLocalFrameB(rope->startJoint, xfB);
            } else {
                destroy_rope_world_anchor(rope, false);
                rope->hasStartAttachment = false;
            }
        }
        if (rope->hasEndAttachment) {
            if (b3Joint_IsValid(rope->endJoint)) {
                b3Transform xfA, xfB;
                xfA.p = get_rope_mount_anchor(scene, rope, true);
                xfA.q = b3Quat_identity;
                xfB.p = b3Vec3_zero;
                xfB.q = b3Quat_identity;
                b3Joint_SetLocalFrameA(rope->endJoint, xfA);
                b3Joint_SetLocalFrameB(rope->endJoint, xfB);
            } else {
                destroy_rope_world_anchor(rope, true);
                rope->hasEndAttachment = false;
            }
        }
    }

    apply_buoyancy_forces(scene);

    // Java uploads the pose at the end of this Sable substep. Let Box derive
    // the kinematic velocity needed to reach that target exactly; setting the
    // pose first and also assigning velocity would integrate the motion twice.
    if (isfinite(timeStep) && timeStep > 0.0) {
        for (int i = 0; i < scene->contraptionCount; i++) {
            KinematicContraption* con = &scene->contraptions[i];
            if (!con->valid || !con->ownsBody || !b3Body_IsValid(con->bodyId)) continue;

            b3WorldTransform target = {
                { con->posePosition[0], con->posePosition[1], con->posePosition[2] },
                con->poseRotation
            };
            b3Body_SetTargetTransform(con->bodyId, target, (float)timeStep, true);
        }
    }

    b3World_Step(scene->worldId, (float)timeStep, scene->solverSubSteps);

    // Update simulated rope point coordinates
    for (int r = 0; r < scene->ropeCount; r++) {
        if (!scene->ropes[r].valid) continue;
        Vox3DRope* rope = &scene->ropes[r];
        for (int p = 0; p < rope->pointCount; p++) {
            if (b3Body_IsValid(rope->segmentBodies[p])) {
                b3WorldTransform xf = b3Body_GetTransform(rope->segmentBodies[p]);
                rope->points[p] = xf.p;
            }
        }
    }

    // Collect contact / hit events
    b3ContactEvents contactEvents = b3World_GetContactEvents(scene->worldId);
    for (int i = 0; i < contactEvents.hitCount && scene->collisionCount < MAX_COLLISIONS; i++) {
        b3ContactHitEvent hit = contactEvents.hitEvents[i];
        ReportedCollision* col = &scene->collisions[scene->collisionCount++];
        
        b3BodyId bA = b3Shape_GetBody(hit.shapeIdA);
        b3BodyId bB = b3Shape_GetBody(hit.shapeIdB);

        SubLevelBody* subA = find_sublevel_by_body(scene, bA);
        SubLevelBody* subB = find_sublevel_by_body(scene, bB);
        if (subA && !subA->isVoxelSubLevel) subA = NULL;
        if (subB && !subB->isVoxelSubLevel) subB = NULL;
        col->idA = subA ? subA->id : -1;
        col->idB = subB ? subB->id : -1;

        // Sable's Rapier event reports a solver contact force, not an
        // approach-speed heuristic. Convert Box's accumulated normal impulse
        // over this Sable step into the closest available force magnitude.
        double totalNormalImpulse = 0.0;
        if (b3Contact_IsValid(hit.contactId)) {
            b3ContactData contact = b3Contact_GetData(hit.contactId);
            for (int manifoldIndex = 0; manifoldIndex < contact.manifoldCount; manifoldIndex++) {
                const b3Manifold* manifold = contact.manifolds + manifoldIndex;
                for (int pointIndex = 0; pointIndex < manifold->pointCount; pointIndex++) {
                    totalNormalImpulse += fmax(0.0, (double)manifold->points[pointIndex].totalNormalImpulse);
                }
            }
        }
        col->forceAmount = isfinite(timeStep) && timeStep > 0.0
            ? totalNormalImpulse / timeStep
            : 0.0;
        b3Vec3 normalA = hit.normal;
        b3Vec3 normalB = b3Neg(hit.normal);
        b3Vec3 pointA = { (float)hit.point.x, (float)hit.point.y, (float)hit.point.z };
        b3Vec3 pointB = pointA;

        if (subA) {
            b3WorldTransform transformA = b3Body_GetTransform(bA);
            normalA = b3InvRotateVector(transformA.q, normalA);
            pointA = b3InvTransformWorldPoint(transformA, hit.point);
        }
        if (subB) {
            b3WorldTransform transformB = b3Body_GetTransform(bB);
            normalB = b3InvRotateVector(transformB.q, normalB);
            pointB = b3InvTransformWorldPoint(transformB, hit.point);
        }

        col->normalA[0] = normalA.x;
        col->normalA[1] = normalA.y;
        col->normalA[2] = normalA.z;
        col->normalB[0] = normalB.x;
        col->normalB[1] = normalB.y;
        col->normalB[2] = normalB.z;
        col->pointA[0] = subA ? pointA.x : hit.point.x;
        col->pointA[1] = subA ? pointA.y : hit.point.y;
        col->pointA[2] = subA ? pointA.z : hit.point.z;
        col->pointB[0] = subB ? pointB.x : hit.point.x;
        col->pointB[1] = subB ? pointB.y : hit.point.y;
        col->pointB[2] = subB ? pointB.z : hit.point.z;
    }

    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_createSubLevel(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint id, jdoubleArray poseArray)
{
    (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    jdouble pose[7];
    (*env)->GetDoubleArrayRegion(env, poseArray, 0, 7, pose);

    VOX_MUTEX_LOCK(scene->mutex);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = (b3Pos){ pose[0], pose[1], pose[2] };
    bodyDef.rotation = make_safe_quat(pose[3], pose[4], pose[5], pose[6]);
    bodyDef.linearDamping = (float)scene->universalDrag;
    bodyDef.angularDamping = (float)scene->universalDrag;
    bodyDef.isBullet = true;
    bodyDef.allowFastRotation = true;
    bodyDef.enableContactRecycling = false;

    b3BodyId bodyId = b3CreateBody(scene->worldId, &bodyDef);

    SubLevelBody* sub = alloc_sublevel(scene, id);
    if (sub) {
        sub->bodyId = bodyId;
        sub->isVoxelSubLevel = true;
        sub->centerOfMass[0] = 0.0;
        sub->centerOfMass[1] = 0.0;
        sub->centerOfMass[2] = 0.0;
        sub->hasMassData = false;
    }

    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_removeSubLevel(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint id)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    SubLevelBody* sub = find_sublevel(scene, id);
    if (sub) {
        release_mounted_contraptions(scene, id);
        invalidate_body_dependents(scene, id);
        for (int i = 0; i < sub->shapeCount; i++) {
            if (b3Shape_IsValid(sub->shapes[i])) {
                b3DestroyShape(sub->shapes[i], false);
            }
        }
        if (sub->shapes) {
            free(sub->shapes);
            sub->shapes = NULL;
        }
        sub->shapeCount = 0;
        sub->shapeCapacity = 0;

        if (sub->voxelData) {
            b3DestroyVoxelData(sub->voxelData);
            sub->voxelData = NULL;
        }
        if (b3Body_IsValid(sub->bodyId)) {
            b3DestroyBody(sub->bodyId);
        }
        sub->valid = false;
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_createBox(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint id, jdouble mass,
    jdouble hx, jdouble hy, jdouble hz, jdoubleArray poseArray)
{
    (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    jdouble pose[7];
    (*env)->GetDoubleArrayRegion(env, poseArray, 0, 7, pose);

    VOX_MUTEX_LOCK(scene->mutex);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = (b3Pos){ pose[0], pose[1], pose[2] };
    bodyDef.rotation = make_safe_quat(pose[3], pose[4], pose[5], pose[6]);
    bodyDef.linearDamping = (float)scene->universalDrag;
    bodyDef.angularDamping = (float)scene->universalDrag;

    b3BodyId bodyId = b3CreateBody(scene->worldId, &bodyDef);

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.density = (float)mass / (float)(8.0 * hx * hy * hz);
    shapeDef.baseMaterial.friction = 0.5f;
    shapeDef.enableHitEvents = true;
    shapeDef.enablePreSolveEvents = true;

    b3BoxHull boxHull = b3MakeBoxHull((float)hx, (float)hy, (float)hz);
    b3ShapeId shapeId = b3CreateHullShape(bodyId, &shapeDef, &boxHull.base);

    b3MassData md;
    md.mass = (float)mass;
    md.center = b3Vec3_zero;
    float ix = (1.0f / 3.0f) * (float)mass * ((float)(hy * hy) + (float)(hz * hz));
    float iy = (1.0f / 3.0f) * (float)mass * ((float)(hx * hx) + (float)(hz * hz));
    float iz = (1.0f / 3.0f) * (float)mass * ((float)(hx * hx) + (float)(hy * hy));
    md.inertia = (b3Matrix3){ { ix, 0.0f, 0.0f }, { 0.0f, iy, 0.0f }, { 0.0f, 0.0f, iz } };
    b3Body_SetMassData(bodyId, md);

    SubLevelBody* sub = alloc_sublevel(scene, id);
    if (sub) {
        sub->bodyId = bodyId;
        sub->isVoxelSubLevel = false;
        sub->shapeCapacity = 1;
        sub->shapes = (b3ShapeId*)malloc(sizeof(b3ShapeId));
        if (sub->shapes) {
            sub->shapes[0] = shapeId;
            sub->shapeCount = 1;
        }
    }

    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_removeBox(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint id)
{
    Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_removeSubLevel(env, clazz, sceneHandle, id);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_getPose(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint id, jdoubleArray storeArray)
{
    (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    b3BodyId bodyId = get_body_by_id(scene, id);
    jdouble store[7] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };
    if (b3Body_IsValid(bodyId)) {
        b3WorldTransform xf = b3Body_GetTransform(bodyId);
        if (isfinite(xf.p.x) && isfinite(xf.p.y) && isfinite(xf.p.z) &&
            isfinite(xf.q.v.x) && isfinite(xf.q.v.y) && isfinite(xf.q.v.z) && isfinite(xf.q.s)) {
            store[0] = xf.p.x;
            store[1] = xf.p.y;
            store[2] = xf.p.z;
            store[3] = xf.q.v.x;
            store[4] = xf.q.v.y;
            store[5] = xf.q.v.z;
            store[6] = xf.q.s;
        } else {
            // NaN recovery: stop velocities on this body
            b3Body_SetLinearVelocity(bodyId, b3Vec3_zero);
            b3Body_SetAngularVelocity(bodyId, b3Vec3_zero);
        }
    }
    VOX_MUTEX_UNLOCK(scene->mutex);

    (*env)->SetDoubleArrayRegion(env, storeArray, 0, 7, store);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_setCenterOfMass(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint id, jdouble x, jdouble y, jdouble z)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    SubLevelBody* sub = find_sublevel(scene, id);
    if (sub) {
        double deltaX = x - sub->centerOfMass[0];
        double deltaY = y - sub->centerOfMass[1];
        double deltaZ = z - sub->centerOfMass[2];
        sub->centerOfMass[0] = x;
        sub->centerOfMass[1] = y;
        sub->centerOfMass[2] = z;
        if (sub->minBounds[0] <= sub->maxBounds[0]) {
            rebuild_sublevel_shapes(scene, sub);
        }
        if (deltaX != 0.0 || deltaY != 0.0 || deltaZ != 0.0) {
            for (int i = 0; i < scene->contraptionCount; i++) {
                KinematicContraption* con = &scene->contraptions[i];
                if (!con->valid || con->ownsBody || con->mountId != id) continue;
                // Preserve the same logical parent-space position while the
                // parent body's origin follows its new center of mass.
                con->posePosition[0] -= deltaX;
                con->posePosition[1] -= deltaY;
                con->posePosition[2] -= deltaZ;
                rebuild_contraption_shapes(con);
            }
        }
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_setLocalBounds(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint id,
    jint minX, jint minY, jint minZ, jint maxX, jint maxY, jint maxZ)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    SubLevelBody* sub = find_sublevel(scene, id);
    if (sub) {
        sub->minBounds[0] = minX; sub->minBounds[1] = minY; sub->minBounds[2] = minZ;
        sub->maxBounds[0] = maxX; sub->maxBounds[1] = maxY; sub->maxBounds[2] = maxZ;
        rebuild_sublevel_shapes(scene, sub);
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
}

static void rebuild_chunk_static_body(Vox3DScene* scene, SceneChunkNode* node) {
    if (!node || !node->isGlobal) return;

    clear_chunk_voxel_data(node);
    if (b3Body_IsValid(node->staticBodyId)) {
        b3DestroyBody(node->staticBodyId);
        node->staticBodyId = b3_nullBodyId;
    }

    int cx = node->cx;
    int cy = node->cy;
    int cz = node->cz;

    b3BodyDef chunkDef = b3DefaultBodyDef();
    chunkDef.type = b3_staticBody;
    chunkDef.position = (b3Pos){ (double)(cx << 4) + 0.5, (double)(cy << 4) + 0.5, (double)(cz << 4) + 0.5 };
    node->staticBodyId = b3CreateBody(scene->worldId, &chunkDef);

    b3Vec3i fullCells[4096];
    int fullCellColliderIds[4096];
    int fullCellCount = 0;

    for (int abx = 0; abx < 16; abx++) {
        for (int abz = 0; abz < 16; abz++) {
            for (int aby = 0; aby < 16; aby++) {
                int p = node->data[abx + (abz << 4) + (aby << 8)];
                int colliderId = (p >> 16) & 0xFFFF;
                if (colliderId <= 0 || colliderId > g_voxelColliderCount) continue;
                VoxelColliderDef* def = &g_voxelColliders[colliderId - 1];
                if (def->isFluid || def->boxCount == 0) continue;

                if (def->boxCount == 1 &&
                    def->boxes[0].min[0] <= 0.001f && def->boxes[0].min[1] <= 0.001f && def->boxes[0].min[2] <= 0.001f &&
                    def->boxes[0].max[0] >= 0.999f && def->boxes[0].max[1] >= 0.999f && def->boxes[0].max[2] >= 0.999f) {
                    fullCells[fullCellCount] = (b3Vec3i){ abx, aby, abz };
                    fullCellColliderIds[fullCellCount++] = colliderId;
                } else {
                    for (int bi = 0; bi < def->boxCount; bi++) {
                        VoxelBox* vb = &def->boxes[bi];
                        float hx = (float)((vb->max[0] - vb->min[0]) * 0.5);
                        float hy = (float)((vb->max[1] - vb->min[1]) * 0.5);
                        float hz = (float)((vb->max[2] - vb->min[2]) * 0.5);
                        if (hx <= 0.0001f || hy <= 0.0001f || hz <= 0.0001f) continue;

                        b3Vec3 offset = {
                            (float)((double)abx + vb->min[0] + (double)hx - 0.5),
                            (float)((double)aby + vb->min[1] + (double)hy - 0.5),
                            (float)((double)abz + vb->min[2] + (double)hz - 0.5)
                        };
                        b3BoxHull boxHull = b3MakeOffsetBoxHull(hx, hy, hz, offset);
                        b3ShapeDef sDef = b3DefaultShapeDef();
                        sDef.baseMaterial.friction = def->friction;
                        sDef.baseMaterial.restitution = def->restitution;
                        sDef.enableHitEvents = true;
                        sDef.updateBodyMass = false;

                        b3CreateHullShape(node->staticBodyId, &sDef, &boxHull.base);
                    }
                }
            }
        }
    }

    bool grouped[4096] = { false };
    b3Vec3i materialCells[4096];
    for (int i = 0; i < fullCellCount; i++) {
        if (grouped[i]) continue;
        int colliderId = fullCellColliderIds[i];
        int materialCellCount = 0;
        for (int j = i; j < fullCellCount; j++) {
            if (!grouped[j] && fullCellColliderIds[j] == colliderId) {
                grouped[j] = true;
                materialCells[materialCellCount++] = fullCells[j];
            }
        }

        b3VoxelData* data = b3CreateVoxelData(materialCells, materialCellCount, 1.0f);
        if (!data) continue;
        if (!add_chunk_voxel_data(node, data)) {
            b3DestroyVoxelData(data);
            continue;
        }

        VoxelColliderDef* material = &g_voxelColliders[colliderId - 1];
        b3ShapeDef sDef = b3DefaultShapeDef();
        sDef.baseMaterial.friction = material->friction;
        sDef.baseMaterial.restitution = material->restitution;
        sDef.enableHitEvents = true;
        sDef.updateBodyMass = false;
        b3CreateVoxelShape(node->staticBodyId, &sDef, data);
    }
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_addChunk(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint cx, jint cy, jint cz,
    jintArray chunkArray, jboolean global, jint id)
{
    (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    jint data[4096];
    (*env)->GetIntArrayRegion(env, chunkArray, 0, 4096, data);

    VOX_MUTEX_LOCK(scene->mutex);

    SceneChunkNode* node = get_or_create_chunk_node(scene, cx, cy, cz);
    if (node) {
        memcpy(node->data, data, sizeof(data));
        node->isGlobal = global ? true : false;
        node->subLevelId = id;

        if (global) {
            rebuild_chunk_static_body(scene, node);
        } else if (id != -1) {
            SubLevelBody* sub = find_sublevel(scene, id);
            if (sub) {
                rebuild_sublevel_shapes(scene, sub);
            }
        }
    }

    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_removeChunk(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint x, jint y, jint z, jboolean global)
{
    (void)env; (void)clazz; (void)global;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    remove_chunk_node(scene, x, y, z);
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_changeBlock(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint x, jint y, jint z, jint newState)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    int cx = x >> 4;
    int cy = y >> 4;
    int cz = z >> 4;
    int bx = x & 15;
    int by = y & 15;
    int bz = z & 15;
    int idx = bx + (bz << 4) + (by << 8);

    SceneChunkNode* node = find_chunk_node(scene, cx, cy, cz);
    if (!node) {
        node = get_or_create_chunk_node(scene, cx, cy, cz);
        if (node) {
            node->isGlobal = true;
            node->subLevelId = -1;
        }
    }
    if (node) {
        node->data[idx] = newState;
        if (node->isGlobal) {
            rebuild_chunk_static_body(scene, node);
        } else {
            SubLevelBody* sub = NULL;
            if (node->subLevelId != -1) {
                sub = find_sublevel(scene, node->subLevelId);
            }
            if (!sub) {
                for (int s = 0; s < scene->subLevelCount; s++) {
                    if (scene->subLevels[s].valid &&
                        x >= scene->subLevels[s].minBounds[0] && x <= scene->subLevels[s].maxBounds[0] &&
                        y >= scene->subLevels[s].minBounds[1] && y <= scene->subLevels[s].maxBounds[1] &&
                        z >= scene->subLevels[s].minBounds[2] && z <= scene->subLevels[s].maxBounds[2]) {
                        sub = &scene->subLevels[s];
                        node->subLevelId = sub->id;
                        break;
                    }
                }
            }
            if (sub) {
                rebuild_sublevel_shapes(scene, sub);
            }
        }
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT jint JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_newVoxelCollider(
    JNIEnv *env, jclass clazz, jdouble friction, jdouble volume, jdouble restitution,
    jboolean isFluid, jobject callback)
{
    (void)env; (void)clazz; (void)callback;
    ensure_global_mutex();
    VOX_MUTEX_LOCK(g_globalMutex);

    if (g_voxelColliderCount >= MAX_VOXEL_COLLIDERS) {
        VOX_MUTEX_UNLOCK(g_globalMutex);
        return -1;
    }

    int idx = g_voxelColliderCount++;
    VoxelColliderDef* def = &g_voxelColliders[idx];
    def->id = idx;
    def->friction = (float)friction;
    def->volume = (float)volume;
    def->restitution = (float)restitution;
    def->isFluid = isFluid ? true : false;
    def->callback = callback ? (*env)->NewGlobalRef(env, callback) : NULL;
    def->boxes = NULL;
    def->boxCount = 0;
    def->boxCapacity = 0;

    VOX_MUTEX_UNLOCK(g_globalMutex);
    return idx;
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_addVoxelColliderBox(
    JNIEnv *env, jclass clazz, jint index, jdoubleArray boundsArray)
{
    (void)clazz;
    ensure_global_mutex();

    jdouble bounds[6];
    (*env)->GetDoubleArrayRegion(env, boundsArray, 0, 6, bounds);

    VOX_MUTEX_LOCK(g_globalMutex);

    if (index < 0 || index >= g_voxelColliderCount) {
        VOX_MUTEX_UNLOCK(g_globalMutex);
        return;
    }

    VoxelColliderDef* def = &g_voxelColliders[index];
    if (def->boxCount == def->boxCapacity) {
        int newCapacity = def->boxCapacity == 0 ? 8 : def->boxCapacity * 2;
        VoxelBox* newBoxes = (VoxelBox*)realloc(def->boxes, newCapacity * sizeof(VoxelBox));
        if (!newBoxes) {
            VOX_MUTEX_UNLOCK(g_globalMutex);
            return;
        }
        def->boxes = newBoxes;
        def->boxCapacity = newCapacity;
    }

    VoxelBox* vb = &def->boxes[def->boxCount++];
    vb->min[0] = bounds[0]; vb->min[1] = bounds[1]; vb->min[2] = bounds[2];
    vb->max[0] = bounds[3]; vb->max[1] = bounds[4]; vb->max[2] = bounds[5];

    VOX_MUTEX_UNLOCK(g_globalMutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_clearVoxelColliderBoxes(
    JNIEnv *env, jclass clazz, jint index)
{
    (void)env; (void)clazz;
    ensure_global_mutex();
    VOX_MUTEX_LOCK(g_globalMutex);
    if (index >= 0 && index < g_voxelColliderCount) {
        g_voxelColliders[index].boxCount = 0;
    }
    VOX_MUTEX_UNLOCK(g_globalMutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_setMassProperties(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint id,
    jdouble mass, jdoubleArray comArray, jdoubleArray inertiaArray)
{
    (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    jdouble com[3];
    (*env)->GetDoubleArrayRegion(env, comArray, 0, 3, com);
    jdouble inertia[9];
    (*env)->GetDoubleArrayRegion(env, inertiaArray, 0, 9, inertia);

    VOX_MUTEX_LOCK(scene->mutex);
    SubLevelBody* sub = find_sublevel(scene, id);
    b3BodyId bodyId = get_body_by_id(scene, id);
    if (b3Body_IsValid(bodyId)) {
        b3MassData md;
        md.mass = (float)mass;
        md.center = b3Vec3_zero;
        md.inertia = (b3Matrix3){
            { (float)inertia[0], (float)inertia[1], (float)inertia[2] },
            { (float)inertia[3], (float)inertia[4], (float)inertia[5] },
            { (float)inertia[6], (float)inertia[7], (float)inertia[8] }
        };
        if (sub) {
            sub->massData = md;
            sub->hasMassData = true;
        }
        b3Body_SetMassData(bodyId, md);
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_teleportObject(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint id,
    jdouble x, jdouble y, jdouble z, jdouble qx, jdouble qy, jdouble qz, jdouble qw)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    b3BodyId bodyId = get_body_by_id(scene, id);
    if (b3Body_IsValid(bodyId)) {
        if (isfinite(x) && isfinite(y) && isfinite(z)) {
            b3Pos pos = { x, y, z };
            b3Quat rot = make_safe_quat(qx, qy, qz, qw);
            b3Body_SetTransform(bodyId, pos, rot);
            b3Body_SetAwake(bodyId, true);
        }
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_wakeUpObject(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint id)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    b3BodyId bodyId = get_body_by_id(scene, id);
    if (b3Body_IsValid(bodyId)) {
        b3Body_SetAwake(bodyId, true);
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_applyForce(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint id,
    jdouble x, jdouble y, jdouble z, jdouble fx, jdouble fy, jdouble fz, jboolean wakeUp)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    b3BodyId bodyId = get_body_by_id(scene, id);
    if (b3Body_IsValid(bodyId)) {
        if (isfinite(x) && isfinite(y) && isfinite(z) && isfinite(fx) && isfinite(fy) && isfinite(fz)) {
            b3WorldTransform xf = b3Body_GetTransform(bodyId);
            b3Vec3 localPoint = { (float)x, (float)y, (float)z };
            b3Vec3 rotPoint = b3RotateVector(xf.q, localPoint);
            b3Pos worldPoint = { xf.p.x + (double)rotPoint.x, xf.p.y + (double)rotPoint.y, xf.p.z + (double)rotPoint.z };
            b3Vec3 localImpulse = { (float)fx, (float)fy, (float)fz };
            b3Vec3 worldImpulse = b3RotateVector(xf.q, localImpulse);
            b3Body_ApplyLinearImpulse(bodyId, worldImpulse, worldPoint, wakeUp ? true : false);
        }
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_applyForceAndTorque(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint id,
    jdouble fx, jdouble fy, jdouble fz, jdouble tx, jdouble ty, jdouble tz, jboolean wakeUp)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    b3BodyId bodyId = get_body_by_id(scene, id);
    if (b3Body_IsValid(bodyId)) {
        if (isfinite(fx) && isfinite(fy) && isfinite(fz) && isfinite(tx) && isfinite(ty) && isfinite(tz)) {
            b3WorldTransform xf = b3Body_GetTransform(bodyId);
            b3Vec3 localForce = { (float)fx, (float)fy, (float)fz };
            b3Vec3 localTorque = { (float)tx, (float)ty, (float)tz };
            b3Vec3 worldForce = b3RotateVector(xf.q, localForce);
            b3Vec3 worldTorque = b3RotateVector(xf.q, localTorque);
            b3Body_ApplyLinearImpulseToCenter(bodyId, worldForce, wakeUp ? true : false);
            b3Body_ApplyAngularImpulse(bodyId, worldTorque, wakeUp ? true : false);
        }
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_addLinearAngularVelocities(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint id,
    jdouble lx, jdouble ly, jdouble lz, jdouble ax, jdouble ay, jdouble az, jboolean wakeUp)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    b3BodyId bodyId = get_body_by_id(scene, id);
    if (b3Body_IsValid(bodyId)) {
        if (isfinite(lx) && isfinite(ly) && isfinite(lz) && isfinite(ax) && isfinite(ay) && isfinite(az)) {
            b3Vec3 curLin = b3Body_GetLinearVelocity(bodyId);
            b3Vec3 curAng = b3Body_GetAngularVelocity(bodyId);
            if (!isfinite(curLin.x) || !isfinite(curLin.y) || !isfinite(curLin.z)) curLin = b3Vec3_zero;
            if (!isfinite(curAng.x) || !isfinite(curAng.y) || !isfinite(curAng.z)) curAng = b3Vec3_zero;
            curLin.x += (float)lx;
            curLin.y += (float)ly;
            curLin.z += (float)lz;
            curAng.x += (float)ax;
            curAng.y += (float)ay;
            curAng.z += (float)az;
            b3Body_SetLinearVelocity(bodyId, curLin);
            b3Body_SetAngularVelocity(bodyId, curAng);
            if (wakeUp) {
                b3Body_SetAwake(bodyId, true);
            }
        }
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_getLinearVelocity(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint id, jdoubleArray storeArray)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    jdouble store[3] = { 0.0, 0.0, 0.0 };
    VOX_MUTEX_LOCK(scene->mutex);
    b3BodyId bodyId = get_body_by_id(scene, id);
    if (b3Body_IsValid(bodyId)) {
        b3Vec3 vel = b3Body_GetLinearVelocity(bodyId);
        if (isfinite(vel.x) && isfinite(vel.y) && isfinite(vel.z)) {
            store[0] = vel.x; store[1] = vel.y; store[2] = vel.z;
        }
    }
    VOX_MUTEX_UNLOCK(scene->mutex);

    (*env)->SetDoubleArrayRegion(env, storeArray, 0, 3, store);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_getAngularVelocity(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint id, jdoubleArray storeArray)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    jdouble store[3] = { 0.0, 0.0, 0.0 };
    VOX_MUTEX_LOCK(scene->mutex);
    b3BodyId bodyId = get_body_by_id(scene, id);
    if (b3Body_IsValid(bodyId)) {
        b3Vec3 vel = b3Body_GetAngularVelocity(bodyId);
        if (isfinite(vel.x) && isfinite(vel.y) && isfinite(vel.z)) {
            store[0] = vel.x; store[1] = vel.y; store[2] = vel.z;
        }
    }
    VOX_MUTEX_UNLOCK(scene->mutex);

    (*env)->SetDoubleArrayRegion(env, storeArray, 0, 3, store);
}

JNIEXPORT jdoubleArray JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_clearCollisions(
    JNIEnv *env, jclass clazz, jlong sceneHandle)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return (*env)->NewDoubleArray(env, 0);
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    int count = scene->collisionCount;
    jdoubleArray result = (*env)->NewDoubleArray(env, count * 15);
    if (count > 0) {
        jdouble* buffer = (jdouble*)malloc(count * 15 * sizeof(jdouble));
        for (int i = 0; i < count; i++) {
            ReportedCollision* col = &scene->collisions[i];
            int base = i * 15;
            buffer[base + 0] = (double)col->idA;
            buffer[base + 1] = (double)col->idB;
            buffer[base + 2] = col->forceAmount;
            buffer[base + 3] = col->normalA[0];
            buffer[base + 4] = col->normalA[1];
            buffer[base + 5] = col->normalA[2];
            buffer[base + 6] = col->normalB[0];
            buffer[base + 7] = col->normalB[1];
            buffer[base + 8] = col->normalB[2];
            buffer[base + 9] = col->pointA[0];
            buffer[base + 10] = col->pointA[1];
            buffer[base + 11] = col->pointA[2];
            buffer[base + 12] = col->pointB[0];
            buffer[base + 13] = col->pointB[1];
            buffer[base + 14] = col->pointB[2];
        }
        (*env)->SetDoubleArrayRegion(env, result, 0, count * 15, buffer);
        free(buffer);
        scene->collisionCount = 0;
    }
    VOX_MUTEX_UNLOCK(scene->mutex);

    return result;
}

// -------------------------------------------------------------------------
// Constraints / Joints
// -------------------------------------------------------------------------

static int64_t g_nextConstraintHandle = 1;

static b3Vec3 get_local_anchor(Vox3DScene* scene, int bodyIdInt, double x, double y, double z) {
    if (bodyIdInt == -1) {
        return (b3Vec3){ (float)x, (float)y, (float)z };
    }
    SubLevelBody* sub = find_sublevel(scene, bodyIdInt);
    if (sub) {
        return (b3Vec3){
            (float)(x - sub->centerOfMass[0]),
            (float)(y - sub->centerOfMass[1]),
            (float)(z - sub->centerOfMass[2])
        };
    }
    KinematicContraption* con = find_contraption(scene, bodyIdInt);
    if (con) {
        return (b3Vec3){
            (float)(x - con->centerOfMass[0]),
            (float)(y - con->centerOfMass[1]),
            (float)(z - con->centerOfMass[2])
        };
    }
    return (b3Vec3){ (float)x, (float)y, (float)z };
}

static b3BodyId get_constraint_body(Vox3DScene* scene, const Vox3DConstraint* c, bool secondSide) {
    int bodyId = secondSide ? c->idB : c->idA;
    if (bodyId != -1) return get_body_by_id(scene, bodyId);

    b3BodyId anchorBody = secondSide ? c->worldAnchorBodyB : c->worldAnchorBodyA;
    return b3Body_IsValid(anchorBody) ? anchorBody : b3_nullBodyId;
}

static bool constraint_backend_valid(Vox3DScene* scene, Vox3DConstraint* c) {
    if (!c || !c->valid) return false;

    b3BodyId bodyA = get_constraint_body(scene, c, false);
    b3BodyId bodyB = get_constraint_body(scene, c, true);
    if (!b3Body_IsValid(bodyA) || !b3Body_IsValid(bodyB)) return false;

    // Free constraints and mask-zero generic constraints intentionally have
    // no solver joint; their optional filter joint only controls contacts.
    if (c->type == 2 || (c->type == 3 && (c->lockedAxesMask & 0x3F) == 0)) {
        return true;
    }
    return b3Joint_IsValid(c->jointId);
}

static void destroy_constraint_resources(Vox3DConstraint* c, bool wakeBodies) {
    if (!c) return;

    if (b3Joint_IsValid(c->jointId)) {
        // Re-enable the pair before destruction so this Vox3D fork buffers the
        // bodies' proxies and allows contacts to be recreated immediately.
        b3Joint_SetCollideConnected(c->jointId, true);
        if (wakeBodies) b3Joint_WakeBodies(c->jointId);
        b3DestroyJoint(c->jointId, wakeBodies);
    }
    c->jointId = b3_nullJointId;

    if (b3Joint_IsValid(c->filterJointId)) {
        b3Joint_SetCollideConnected(c->filterJointId, true);
        if (wakeBodies) b3Joint_WakeBodies(c->filterJointId);
        b3DestroyJoint(c->filterJointId, wakeBodies);
    }
    c->filterJointId = b3_nullJointId;

    if (b3Body_IsValid(c->worldAnchorBodyA)) b3DestroyBody(c->worldAnchorBodyA);
    if (b3Body_IsValid(c->worldAnchorBodyB)) b3DestroyBody(c->worldAnchorBodyB);
    c->worldAnchorBodyA = b3_nullBodyId;
    c->worldAnchorBodyB = b3_nullBodyId;
    memset(c->impulses, 0, sizeof(c->impulses));
}

static b3Vec3 get_constraint_local_anchor(Vox3DScene* scene, const Vox3DConstraint* c, bool secondSide) {
    int bodyId = secondSide ? c->idB : c->idA;
    if (bodyId == -1) return b3Vec3_zero;

    const double* source = secondSide ? c->sourcePos2 : c->sourcePos1;
    return get_local_anchor(scene, bodyId, source[0], source[1], source[2]);
}

static void move_constraint_world_anchor(const Vox3DConstraint* c, bool secondSide) {
    int bodyId = secondSide ? c->idB : c->idA;
    if (bodyId != -1) return;

    b3BodyId anchorBody = secondSide ? c->worldAnchorBodyB : c->worldAnchorBodyA;
    const double* source = secondSide ? c->sourcePos2 : c->sourcePos1;
    if (b3Body_IsValid(anchorBody)) {
        b3Body_SetTransform(anchorBody, (b3Pos){ source[0], source[1], source[2] }, b3Quat_identity);
    }
}

static bool resolve_constraint_bodies(
    Vox3DScene* scene,
    int idA, double ax, double ay, double az,
    int idB, double bx, double by, double bz,
    b3BodyId* bodyA, b3BodyId* bodyB,
    b3BodyId* worldAnchorA, b3BodyId* worldAnchorB)
{
    *worldAnchorA = b3_nullBodyId;
    *worldAnchorB = b3_nullBodyId;
    *bodyA = idA == -1 ? create_world_anchor_body(scene, ax, ay, az) : get_body_by_id(scene, idA);
    if (idA == -1) *worldAnchorA = *bodyA;
    *bodyB = idB == -1 ? create_world_anchor_body(scene, bx, by, bz) : get_body_by_id(scene, idB);
    if (idB == -1) *worldAnchorB = *bodyB;

    if (b3Body_IsValid(*bodyA) && b3Body_IsValid(*bodyB)) return true;

    if (b3Body_IsValid(*worldAnchorA)) b3DestroyBody(*worldAnchorA);
    if (b3Body_IsValid(*worldAnchorB)) b3DestroyBody(*worldAnchorB);
    *bodyA = b3_nullBodyId;
    *bodyB = b3_nullBodyId;
    *worldAnchorA = b3_nullBodyId;
    *worldAnchorB = b3_nullBodyId;
    return false;
}

static void sync_constraint_contact_filter(Vox3DScene* scene, Vox3DConstraint* c) {
    if (!c || !c->valid) return;

    if (b3Joint_IsValid(c->jointId)) {
        if (b3Joint_IsValid(c->filterJointId)) {
            b3Joint_SetCollideConnected(c->filterJointId, true);
            b3DestroyJoint(c->filterJointId, true);
            c->filterJointId = b3_nullJointId;
        }
        b3Joint_SetCollideConnected(c->jointId, c->contactsEnabled);
        return;
    }

    if (c->contactsEnabled) {
        if (b3Joint_IsValid(c->filterJointId)) {
            b3Joint_SetCollideConnected(c->filterJointId, true);
            b3DestroyJoint(c->filterJointId, true);
            c->filterJointId = b3_nullJointId;
        }
        return;
    }

    if (b3Joint_IsValid(c->filterJointId)) return;
    b3BodyId bodyA = get_constraint_body(scene, c, false);
    b3BodyId bodyB = get_constraint_body(scene, c, true);
    if (!b3Body_IsValid(bodyA) || !b3Body_IsValid(bodyB)) return;

    b3FilterJointDef def = b3DefaultFilterJointDef();
    def.base.bodyIdA = bodyA;
    def.base.bodyIdB = bodyB;
    // This Vox3D fork only destroys already-existing contacts when the value
    // changes through the setter. Create as true, then toggle to false.
    def.base.collideConnected = true;
    c->filterJointId = b3CreateFilterJoint(scene->worldId, &def);
    if (b3Joint_IsValid(c->filterJointId)) {
        b3Joint_SetCollideConnected(c->filterJointId, false);
    }
}

static double clamp_motor_output(const Vox3DConstraint* c, int axis, double value) {
    if (!isfinite(value)) return 0.0;
    if (c->motorHasForceLimit[axis] && isfinite(c->motorMaxForce[axis])) {
        double limit = fmax(0.0, c->motorMaxForce[axis]);
        return fmax(-limit, fmin(limit, value));
    }
    return value;
}

static double wrapped_motor_error(double target, double current) {
    if (!isfinite(target) || !isfinite(current)) return 0.0;
    const double twoPi = 6.28318530717958647692;
    const double pi = 3.14159265358979323846;
    double error = fmod(target - current, twoPi);
    if (error > pi) error -= twoPi;
    if (error < -pi) error += twoPi;
    return error;
}

static double implicit_motor_acceleration(
    double error, double velocity, double stiffness, double damping, double timeStep)
{
    if (!isfinite(timeStep) || timeStep <= 0.0) return 0.0;
    // Rapier's default AccelerationBased motor coefficients reduce to this
    // implicit scalar PD update. The extra stiffness*dt velocity term and
    // denominator keep high-stiffness addon motors stable at tick time steps.
    double denominator = 1.0 + timeStep * damping +
        timeStep * timeStep * stiffness;
    if (!isfinite(denominator) || denominator <= 1e-12) return 0.0;
    return (stiffness * error - (damping + timeStep * stiffness) * velocity) /
        denominator;
}

static float body_inverse_mass(b3BodyId body) {
    if (!b3Body_IsValid(body) || b3Body_GetType(body) != b3_dynamicBody) return 0.0f;
    float mass = b3Body_GetMass(body);
    return isfinite(mass) && mass > FLT_EPSILON ? 1.0f / mass : 0.0f;
}

static b3Vec3 body_world_inverse_inertia_mul(b3BodyId body, b3Vec3 worldVector) {
    if (!b3Body_IsValid(body) || b3Body_GetType(body) != b3_dynamicBody) {
        return b3Vec3_zero;
    }

    b3MassData massData = b3Body_GetMassData(body);
    b3Matrix3 inverseLocalInertia = b3InvertMatrix(massData.inertia);
    b3Quat rotation = b3Body_GetRotation(body);
    b3Vec3 localVector = b3InvRotateVector(rotation, worldVector);
    return b3RotateVector(rotation, b3MulMV(inverseLocalInertia, localVector));
}

static double linear_motor_effective_mass(
    b3BodyId bodyA, b3BodyId bodyB,
    b3Pos worldAnchorA, b3Pos worldAnchorB, b3Vec3 worldAxis)
{
    double inverseEffectiveMass = body_inverse_mass(bodyA) + body_inverse_mass(bodyB);

    if (b3Body_GetType(bodyA) == b3_dynamicBody) {
        b3Vec3 radius = b3SubPos(worldAnchorA, b3Body_GetWorldCenter(bodyA));
        b3Vec3 angularJacobian = b3Cross(radius, worldAxis);
        inverseEffectiveMass += b3Dot(
            angularJacobian,
            body_world_inverse_inertia_mul(bodyA, angularJacobian));
    }
    if (b3Body_GetType(bodyB) == b3_dynamicBody) {
        b3Vec3 radius = b3SubPos(worldAnchorB, b3Body_GetWorldCenter(bodyB));
        b3Vec3 angularJacobian = b3Cross(radius, worldAxis);
        inverseEffectiveMass += b3Dot(
            angularJacobian,
            body_world_inverse_inertia_mul(bodyB, angularJacobian));
    }

    return isfinite(inverseEffectiveMass) && inverseEffectiveMass > 1e-12
        ? 1.0 / inverseEffectiveMass
        : 0.0;
}

static double angular_motor_effective_mass(
    b3BodyId bodyA, b3BodyId bodyB, b3Vec3 worldAxis)
{
    double inverseEffectiveMass = b3Dot(
        worldAxis, body_world_inverse_inertia_mul(bodyA, worldAxis));
    inverseEffectiveMass += b3Dot(
        worldAxis, body_world_inverse_inertia_mul(bodyB, worldAxis));
    return isfinite(inverseEffectiveMass) && inverseEffectiveMass > 1e-12
        ? 1.0 / inverseEffectiveMass
        : 0.0;
}

static void apply_constraint_pd_motors(
    Vox3DScene* scene, Vox3DConstraint* c, double timeStep, int motorAxesMask)
{
    memset(c->impulses, 0, sizeof(c->impulses));

    bool hasMotor = false;
    for (int axis = 0; axis < 6; axis++) {
        if ((motorAxesMask & (1 << axis)) != 0 && c->motorConfigured[axis] &&
            (c->motorStiffness[axis] > 0.0 || c->motorDamping[axis] > 0.0)) {
            hasMotor = true;
            break;
        }
    }
    if (!hasMotor) return;

    b3BodyId bodyA = get_constraint_body(scene, c, false);
    b3BodyId bodyB = get_constraint_body(scene, c, true);
    if (!b3Body_IsValid(bodyA) || !b3Body_IsValid(bodyB)) return;

    b3WorldTransform transformA = b3Body_GetTransform(bodyA);
    b3WorldTransform transformB = b3Body_GetTransform(bodyB);
    b3Quat frameRotationA = b3MulQuat(transformA.q, c->rot1);
    b3Quat frameRotationB = b3MulQuat(transformB.q, c->rot2);
    b3Pos worldAnchorA = b3Body_GetWorldPoint(bodyA, c->pos1);
    b3Pos worldAnchorB = b3Body_GetWorldPoint(bodyB, c->pos2);

    b3Vec3 linearPosition = b3InvRotateVector(
        frameRotationA, b3SubPos(worldAnchorB, worldAnchorA));
    b3Vec3 relativeLinearVelocity = b3InvRotateVector(frameRotationA,
        b3Sub(b3Body_GetWorldPointVelocity(bodyB, worldAnchorB),
              b3Body_GetWorldPointVelocity(bodyA, worldAnchorA)));
    b3Vec3 relativeAngularVelocity = b3InvRotateVector(frameRotationA,
        b3Sub(b3Body_GetAngularVelocity(bodyB), b3Body_GetAngularVelocity(bodyA)));

    b3Quat relativeRotation = b3InvMulQuat(frameRotationA, frameRotationB);
    if (relativeRotation.s < 0.0f) relativeRotation = b3NegateQuat(relativeRotation);

    double linearPositionValues[3] = { linearPosition.x, linearPosition.y, linearPosition.z };
    double linearVelocityValues[3] = {
        relativeLinearVelocity.x, relativeLinearVelocity.y, relativeLinearVelocity.z
    };
    double angularPositionValues[3] = {
        2.0 * asin(fmax(-1.0, fmin(1.0, relativeRotation.v.x))),
        2.0 * asin(fmax(-1.0, fmin(1.0, relativeRotation.v.y))),
        2.0 * asin(fmax(-1.0, fmin(1.0, relativeRotation.v.z)))
    };
    double angularVelocityValues[3] = {
        relativeAngularVelocity.x, relativeAngularVelocity.y, relativeAngularVelocity.z
    };
    b3Vec3 localForce = b3Vec3_zero;
    b3Vec3 localTorque = b3Vec3_zero;
    float* forceComponents = &localForce.x;
    float* torqueComponents = &localTorque.x;

    for (int axis = 0; axis < 3; axis++) {
        b3Vec3 localAxis = axis == 0 ? b3Vec3_axisX :
            (axis == 1 ? b3Vec3_axisY : b3Vec3_axisZ);
        b3Vec3 worldAxis = b3RotateVector(frameRotationA, localAxis);

        if ((motorAxesMask & (1 << axis)) != 0 && c->motorConfigured[axis]) {
            double acceleration = implicit_motor_acceleration(
                c->motorTarget[axis] - linearPositionValues[axis],
                linearVelocityValues[axis],
                c->motorStiffness[axis], c->motorDamping[axis], timeStep);
            double output = linear_motor_effective_mass(
                bodyA, bodyB, worldAnchorA, worldAnchorB, worldAxis) * acceleration;
            forceComponents[axis] = (float)clamp_motor_output(c, axis, output);
        }

        int angularIndex = axis + 3;
        if ((motorAxesMask & (1 << angularIndex)) != 0 && c->motorConfigured[angularIndex]) {
            double acceleration = implicit_motor_acceleration(
                wrapped_motor_error(c->motorTarget[angularIndex], angularPositionValues[axis]),
                angularVelocityValues[axis], c->motorStiffness[angularIndex],
                c->motorDamping[angularIndex], timeStep);
            double output = angular_motor_effective_mass(bodyA, bodyB, worldAxis) * acceleration;
            torqueComponents[axis] = (float)clamp_motor_output(c, angularIndex, output);
        }
    }

    b3Vec3 worldForce = b3RotateVector(frameRotationA, localForce);
    b3Vec3 worldTorque = b3RotateVector(frameRotationA, localTorque);
    double safeTimeStep = isfinite(timeStep) && timeStep > 0.0 ? timeStep : 0.0;
    c->impulses[0] = (float)(worldForce.x * safeTimeStep);
    c->impulses[1] = (float)(worldForce.y * safeTimeStep);
    c->impulses[2] = (float)(worldForce.z * safeTimeStep);
    c->impulses[3] = (float)(worldTorque.x * safeTimeStep);
    c->impulses[4] = (float)(worldTorque.y * safeTimeStep);
    c->impulses[5] = (float)(worldTorque.z * safeTimeStep);

    if (b3LengthSquared(worldForce) > 1e-12f || b3LengthSquared(worldTorque) > 1e-12f) {
        if (b3Body_GetType(bodyA) == b3_dynamicBody) b3Body_SetAwake(bodyA, true);
        if (b3Body_GetType(bodyB) == b3_dynamicBody) b3Body_SetAwake(bodyB, true);
    }
    b3Body_ApplyForce(bodyA, b3Neg(worldForce), worldAnchorA, false);
    b3Body_ApplyForce(bodyB, worldForce, worldAnchorB, false);
    b3Body_ApplyTorque(bodyA, b3Neg(worldTorque), false);
    b3Body_ApplyTorque(bodyB, worldTorque, false);
}

static void apply_rotary_constraint_motor(Vox3DScene* scene, Vox3DConstraint* c, double timeStep) {
    c->impulses[3] = 0.0f;
    c->impulses[4] = 0.0f;
    c->impulses[5] = 0.0f;
    if (!c->motorConfigured[3] ||
        (c->motorStiffness[3] <= 0.0 && c->motorDamping[3] <= 0.0) ||
        !b3Joint_IsValid(c->jointId)) {
        return;
    }

    b3BodyId bodyA = get_constraint_body(scene, c, false);
    b3BodyId bodyB = get_constraint_body(scene, c, true);
    if (!b3Body_IsValid(bodyA) || !b3Body_IsValid(bodyB)) return;

    b3Quat frameRotationA = b3MulQuat(b3Body_GetRotation(bodyA), c->rot1);
    b3Vec3 worldAxis = b3RotateVector(frameRotationA, b3Vec3_axisZ);
    b3Vec3 relativeAngularVelocity = b3Sub(
        b3Body_GetAngularVelocity(bodyB), b3Body_GetAngularVelocity(bodyA));
    double angle = b3RevoluteJoint_GetAngle(c->jointId);
    double angularSpeed = b3Dot(relativeAngularVelocity, worldAxis);
    double acceleration = implicit_motor_acceleration(
        wrapped_motor_error(c->motorTarget[3], angle), angularSpeed,
        c->motorStiffness[3], c->motorDamping[3], timeStep);
    double output = angular_motor_effective_mass(bodyA, bodyB, worldAxis) * acceleration;
    output = clamp_motor_output(c, 3, output);
    b3Vec3 torque = b3MulSV((float)output, worldAxis);

    double safeTimeStep = isfinite(timeStep) && timeStep > 0.0 ? timeStep : 0.0;
    c->impulses[3] = (float)(torque.x * safeTimeStep);
    c->impulses[4] = (float)(torque.y * safeTimeStep);
    c->impulses[5] = (float)(torque.z * safeTimeStep);

    if (b3LengthSquared(torque) > 1e-12f) {
        if (b3Body_GetType(bodyA) == b3_dynamicBody) b3Body_SetAwake(bodyA, true);
        if (b3Body_GetType(bodyB) == b3_dynamicBody) b3Body_SetAwake(bodyB, true);
    }
    b3Body_ApplyTorque(bodyA, b3Neg(torque), false);
    b3Body_ApplyTorque(bodyB, torque, false);
}

JNIEXPORT jlong JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_addRotaryConstraint(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint idA, jint idB,
    jdouble lxa, jdouble lya, jdouble lza, jdouble lxb, jdouble lyb, jdouble lzb,
    jdouble axa, jdouble aya, jdouble aza, jdouble axb, jdouble ayb, jdouble azb)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return 0;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    b3BodyId bA, bB, worldAnchorA, worldAnchorB;
    if (!resolve_constraint_bodies(scene,
        idA, lxa, lya, lza, idB, lxb, lyb, lzb,
        &bA, &bB, &worldAnchorA, &worldAnchorB)) {
        VOX_MUTEX_UNLOCK(scene->mutex);
        return 0;
    }

    b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
    def.base.bodyIdA = bA;
    def.base.bodyIdB = bB;
    def.base.localFrameA.p = idA == -1 ? b3Vec3_zero : get_local_anchor(scene, idA, lxa, lya, lza);
    def.base.localFrameB.p = idB == -1 ? b3Vec3_zero : get_local_anchor(scene, idB, lxb, lyb, lzb);
    b3Vec3 axisA = { (float)axa, (float)aya, (float)aza };
    b3Vec3 axisB = { (float)axb, (float)ayb, (float)azb };
    if (b3LengthSquared(axisA) > 1e-8f) {
        axisA = b3Normalize(axisA);
        def.base.localFrameA.q = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisZ, axisA);
    }
    if (b3LengthSquared(axisB) > 1e-8f) {
        axisB = b3Normalize(axisB);
        def.base.localFrameB.q = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisZ, axisB);
    }
    def.base.collideConnected = true;

    b3JointId jointId = b3CreateRevoluteJoint(scene->worldId, &def);
    if (!b3Joint_IsValid(jointId)) {
        if (b3Body_IsValid(worldAnchorA)) b3DestroyBody(worldAnchorA);
        if (b3Body_IsValid(worldAnchorB)) b3DestroyBody(worldAnchorB);
        VOX_MUTEX_UNLOCK(scene->mutex);
        return 0;
    }
    b3Joint_WakeBodies(jointId);
    int64_t handle = g_nextConstraintHandle++;

    Vox3DConstraint* c = alloc_constraint(scene, handle);
    if (c) {
        c->idA = idA;
        c->idB = idB;
        c->jointId = jointId;
        c->worldAnchorBodyA = worldAnchorA;
        c->worldAnchorBodyB = worldAnchorB;
        c->type = 1; // rotary
        c->jointKind = 3;
        c->sourcePos1[0] = lxa;
        c->sourcePos1[1] = lya;
        c->sourcePos1[2] = lza;
        c->sourcePos2[0] = lxb;
        c->sourcePos2[1] = lyb;
        c->sourcePos2[2] = lzb;
        c->pos1 = def.base.localFrameA.p;
        c->pos2 = def.base.localFrameB.p;
        c->normal1 = axisA;
        c->normal2 = axisB;
        c->rot1 = def.base.localFrameA.q;
        c->rot2 = def.base.localFrameB.q;
        // Match Rapier's RotaryConstraint default.
        c->contactsEnabled = true;
        sync_constraint_contact_filter(scene, c);
    } else {
        if (b3Joint_IsValid(jointId)) b3DestroyJoint(jointId, true);
        if (b3Body_IsValid(worldAnchorA)) b3DestroyBody(worldAnchorA);
        if (b3Body_IsValid(worldAnchorB)) b3DestroyBody(worldAnchorB);
        handle = 0;
    }

    VOX_MUTEX_UNLOCK(scene->mutex);
    return handle;
}

JNIEXPORT jlong JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_addFixedConstraint(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint idA, jint idB,
    jdouble lxa, jdouble lya, jdouble lza, jdouble lxb, jdouble lyb, jdouble lzb,
    jdouble qx, jdouble qy, jdouble qz, jdouble qw)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return 0;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    b3BodyId bA, bB, worldAnchorA, worldAnchorB;
    if (!resolve_constraint_bodies(scene,
        idA, lxa, lya, lza, idB, lxb, lyb, lzb,
        &bA, &bB, &worldAnchorA, &worldAnchorB)) {
        VOX_MUTEX_UNLOCK(scene->mutex);
        return 0;
    }

    b3Quat quat = make_safe_quat(qx, qy, qz, qw);

    b3WeldJointDef def = b3DefaultWeldJointDef();
    def.base.bodyIdA = bA;
    def.base.bodyIdB = bB;
    def.base.localFrameA.p = idA == -1 ? b3Vec3_zero : get_local_anchor(scene, idA, lxa, lya, lza);
    def.base.localFrameA.q = quat;
    def.base.localFrameB.p = idB == -1 ? b3Vec3_zero : get_local_anchor(scene, idB, lxb, lyb, lzb);
    def.base.localFrameB.q = b3Quat_identity;
    def.linearHertz = 0.0f; // rigid
    def.linearDampingRatio = 0.0f;
    def.angularHertz = 0.0f; // rigid
    def.angularDampingRatio = 0.0f;
    def.base.collideConnected = true;

    b3JointId jointId = b3CreateWeldJoint(scene->worldId, &def);
    if (!b3Joint_IsValid(jointId)) {
        if (b3Body_IsValid(worldAnchorA)) b3DestroyBody(worldAnchorA);
        if (b3Body_IsValid(worldAnchorB)) b3DestroyBody(worldAnchorB);
        VOX_MUTEX_UNLOCK(scene->mutex);
        return 0;
    }
    b3Joint_WakeBodies(jointId);
    b3Body_SetAwake(bA, true);
    b3Body_SetAwake(bB, true);
    int64_t handle = g_nextConstraintHandle++;

    Vox3DConstraint* c = alloc_constraint(scene, handle);
    if (c) {
        c->idA = idA;
        c->idB = idB;
        c->jointId = jointId;
        c->worldAnchorBodyA = worldAnchorA;
        c->worldAnchorBodyB = worldAnchorB;
        c->type = 0; // fixed
        c->jointKind = 1;
        c->sourcePos1[0] = lxa;
        c->sourcePos1[1] = lya;
        c->sourcePos1[2] = lza;
        c->sourcePos2[0] = lxb;
        c->sourcePos2[1] = lyb;
        c->sourcePos2[2] = lzb;
        c->pos1 = def.base.localFrameA.p;
        c->rot1 = quat;
        c->pos2 = def.base.localFrameB.p;
        c->rot2 = b3Quat_identity;
        // Match Rapier's FixedConstraint default.
        c->contactsEnabled = false;
        sync_constraint_contact_filter(scene, c);
    } else {
        if (b3Joint_IsValid(jointId)) b3DestroyJoint(jointId, true);
        if (b3Body_IsValid(worldAnchorA)) b3DestroyBody(worldAnchorA);
        if (b3Body_IsValid(worldAnchorB)) b3DestroyBody(worldAnchorB);
        handle = 0;
    }

    VOX_MUTEX_UNLOCK(scene->mutex);
    return handle;
}

JNIEXPORT jlong JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_addFreeConstraint(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint idA, jint idB,
    jdouble lxa, jdouble lya, jdouble lza, jdouble lxb, jdouble lyb, jdouble lzb,
    jdouble qx, jdouble qy, jdouble qz, jdouble qw)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return 0;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    b3BodyId bA, bB, worldAnchorA, worldAnchorB;
    if (!resolve_constraint_bodies(scene,
        idA, lxa, lya, lza, idB, lxb, lyb, lzb,
        &bA, &bB, &worldAnchorA, &worldAnchorB)) {
        VOX_MUTEX_UNLOCK(scene->mutex);
        return 0;
    }

    b3Quat quat = make_safe_quat(qx, qy, qz, qw);
    int64_t handle = g_nextConstraintHandle++;

    Vox3DConstraint* c = alloc_constraint(scene, handle);
    if (c) {
        c->idA = idA;
        c->idB = idB;
        c->jointId = b3_nullJointId;
        c->worldAnchorBodyA = worldAnchorA;
        c->worldAnchorBodyB = worldAnchorB;
        c->type = 2; // free; configured motors are applied per axis before each step
        c->jointKind = 0;
        c->sourcePos1[0] = lxa;
        c->sourcePos1[1] = lya;
        c->sourcePos1[2] = lza;
        c->sourcePos2[0] = lxb;
        c->sourcePos2[1] = lyb;
        c->sourcePos2[2] = lzb;
        c->pos1 = idA == -1 ? b3Vec3_zero : get_local_anchor(scene, idA, lxa, lya, lza);
        c->rot1 = quat;
        c->pos2 = idB == -1 ? b3Vec3_zero : get_local_anchor(scene, idB, lxb, lyb, lzb);
        c->rot2 = b3Quat_identity;
        // Match Rapier's FreeConstraint default.
        c->contactsEnabled = true;
        sync_constraint_contact_filter(scene, c);
    } else {
        if (b3Body_IsValid(worldAnchorA)) b3DestroyBody(worldAnchorA);
        if (b3Body_IsValid(worldAnchorB)) b3DestroyBody(worldAnchorB);
        handle = 0;
    }

    VOX_MUTEX_UNLOCK(scene->mutex);
    return handle;
}

static b3Vec3 axis_for_index(int axis) {
    if (axis == 0) return b3Vec3_axisX;
    if (axis == 1) return b3Vec3_axisY;
    return b3Vec3_axisZ;
}

static int single_axis_index(int mask) {
    if (mask == 1) return 0;
    if (mask == 2) return 1;
    if (mask == 4) return 2;
    return -1;
}

static bool generic_mask_supported(int mask) {
    mask &= 0x3F;
    int linearMask = mask & 0x7;
    int angularMask = (mask >> 3) & 0x7;

    // Empty, ball/socket (optionally locking one or two angular axes), weld,
    // and slider configurations have exact Box3D equivalents.
    if (mask == 0 || linearMask == 0x7) return true;
    return angularMask == 0x7 && single_axis_index((~linearMask) & 0x7) >= 0;
}

static void rebuild_generic_joint(Vox3DScene* scene, Vox3DConstraint* c) {
    if (b3Joint_IsValid(c->jointId)) {
        b3DestroyJoint(c->jointId, true);
    }
    c->jointId = b3_nullJointId;
    c->jointKind = 0;

    b3BodyId bodyA = get_constraint_body(scene, c, false);
    b3BodyId bodyB = get_constraint_body(scene, c, true);
    if (!b3Body_IsValid(bodyA) || !b3Body_IsValid(bodyB)) return;

    // Rebuilds can happen after either body's center of mass has moved. Always
    // derive the Box3D-local frames from Sable's original double coordinates.
    c->pos1 = get_constraint_local_anchor(scene, c, false);
    c->pos2 = get_constraint_local_anchor(scene, c, true);

    int mask = c->lockedAxesMask & 0x3F;
    int linearMask = mask & 0x7;
    int angularMask = (mask >> 3) & 0x7;

    if (mask == 0x3F) {
        b3WeldJointDef def = b3DefaultWeldJointDef();
        def.base.bodyIdA = bodyA;
        def.base.bodyIdB = bodyB;
        def.base.localFrameA = (b3Transform){ c->pos1, c->rot1 };
        def.base.localFrameB = (b3Transform){ c->pos2, c->rot2 };
        def.base.collideConnected = true;
        def.linearHertz = 0.0f;
        def.angularHertz = 0.0f;
        c->jointId = b3CreateWeldJoint(scene->worldId, &def);
        c->jointKind = 1;
    } else if (linearMask == 0x7 && single_axis_index((~angularMask) & 0x7) >= 0) {
        int freeAxis = single_axis_index((~angularMask) & 0x7);
        b3Quat alignment = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisZ, axis_for_index(freeAxis));
        b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
        def.base.bodyIdA = bodyA;
        def.base.bodyIdB = bodyB;
        def.base.localFrameA = (b3Transform){ c->pos1, b3MulQuat(c->rot1, alignment) };
        def.base.localFrameB = (b3Transform){ c->pos2, b3MulQuat(c->rot2, alignment) };
        def.base.collideConnected = true;
        c->jointId = b3CreateRevoluteJoint(scene->worldId, &def);
        c->jointKind = 3;
    } else if (angularMask == 0x7 && single_axis_index((~linearMask) & 0x7) >= 0) {
        int freeAxis = single_axis_index((~linearMask) & 0x7);
        b3Quat alignment = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisX, axis_for_index(freeAxis));
        b3PrismaticJointDef def = b3DefaultPrismaticJointDef();
        def.base.bodyIdA = bodyA;
        def.base.bodyIdB = bodyB;
        def.base.localFrameA = (b3Transform){ c->pos1, b3MulQuat(c->rot1, alignment) };
        def.base.localFrameB = (b3Transform){ c->pos2, b3MulQuat(c->rot2, alignment) };
        def.base.collideConnected = true;
        c->jointId = b3CreatePrismaticJoint(scene->worldId, &def);
        c->jointKind = 4;
    } else if (linearMask == 0x7 &&
               (angularMask == 0 || single_axis_index(angularMask) >= 0)) {
        int lockedAngularAxis = single_axis_index(angularMask);
        b3Quat alignment = lockedAngularAxis >= 0
            ? b3ComputeQuatBetweenUnitVectors(b3Vec3_axisZ, axis_for_index(lockedAngularAxis))
            : b3Quat_identity;
        b3SphericalJointDef def = b3DefaultSphericalJointDef();
        def.base.bodyIdA = bodyA;
        def.base.bodyIdB = bodyB;
        def.base.localFrameA = (b3Transform){ c->pos1, b3MulQuat(c->rot1, alignment) };
        def.base.localFrameB = (b3Transform){ c->pos2, b3MulQuat(c->rot2, alignment) };
        def.base.collideConnected = true;
        c->jointId = b3CreateSphericalJoint(scene->worldId, &def);
        c->jointKind = 2;
        if (lockedAngularAxis >= 0 && b3Joint_IsValid(c->jointId)) {
            b3SphericalJoint_SetTwistLimits(c->jointId, 0.0f, 0.0f);
            b3SphericalJoint_EnableTwistLimit(c->jointId, true);
        }
    }

    if (b3Joint_IsValid(c->jointId)) {
        b3Joint_WakeBodies(c->jointId);
    }
    sync_constraint_contact_filter(scene, c);
}

JNIEXPORT jlong JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_addGenericConstraint(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint idA, jint idB,
    jdouble lxa, jdouble lya, jdouble lza, jdouble qxa, jdouble qya, jdouble qza, jdouble qwa,
    jdouble lxb, jdouble lyb, jdouble lzb, jdouble qxb, jdouble qyb, jdouble qzb, jdouble qwb,
    jint lockedAxesMask)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return 0;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    if (!generic_mask_supported(lockedAxesMask)) {
        VOX_MUTEX_UNLOCK(scene->mutex);
        return 0;
    }
    b3BodyId bA, bB, worldAnchorA, worldAnchorB;
    if (!resolve_constraint_bodies(scene,
        idA, lxa, lya, lza, idB, lxb, lyb, lzb,
        &bA, &bB, &worldAnchorA, &worldAnchorB)) {
        VOX_MUTEX_UNLOCK(scene->mutex);
        return 0;
    }

    b3Quat qa = make_safe_quat(qxa, qya, qza, qwa);
    b3Quat qb = make_safe_quat(qxb, qyb, qzb, qwb);

    int64_t handle = g_nextConstraintHandle++;
    Vox3DConstraint* c = alloc_constraint(scene, handle);
    if (c) {
        c->idA = idA;
        c->idB = idB;
        c->jointId = b3_nullJointId;
        c->worldAnchorBodyA = worldAnchorA;
        c->worldAnchorBodyB = worldAnchorB;
        c->type = 3;
        c->jointKind = 0;
        c->sourcePos1[0] = lxa;
        c->sourcePos1[1] = lya;
        c->sourcePos1[2] = lza;
        c->sourcePos2[0] = lxb;
        c->sourcePos2[1] = lyb;
        c->sourcePos2[2] = lzb;
        c->pos1 = idA == -1 ? b3Vec3_zero : get_local_anchor(scene, idA, lxa, lya, lza);
        c->rot1 = qa;
        c->pos2 = idB == -1 ? b3Vec3_zero : get_local_anchor(scene, idB, lxb, lyb, lzb);
        c->rot2 = qb;
        c->lockedAxesMask = lockedAxesMask & 0x3F;
        // Match Rapier's GenericConstraint default.
        c->contactsEnabled = true;
        rebuild_generic_joint(scene, c);
    } else {
        if (b3Body_IsValid(worldAnchorA)) b3DestroyBody(worldAnchorA);
        if (b3Body_IsValid(worldAnchorB)) b3DestroyBody(worldAnchorB);
        handle = 0;
    }

    VOX_MUTEX_UNLOCK(scene->mutex);
    return handle;
}

static void update_joint_frames(Vox3DScene* scene, Vox3DConstraint* c, bool wakeBodies) {
    if (!c) return;

    c->pos1 = get_constraint_local_anchor(scene, c, false);
    c->pos2 = get_constraint_local_anchor(scene, c, true);

    if (!b3Joint_IsValid(c->jointId)) return;

    if (c->type == 0 || c->type == 2 || c->type == 3) { // Weld / Free / Generic
        b3Transform xfA, xfB;
        xfA.p = c->pos1;
        xfA.q = c->rot1;

        xfB.p = c->pos2;
        xfB.q = c->rot2;

        if (c->type == 3 && c->jointKind == 2) {
            int lockedAxis = single_axis_index((c->lockedAxesMask >> 3) & 0x7);
            if (lockedAxis >= 0) {
                b3Quat alignment = b3ComputeQuatBetweenUnitVectors(
                    b3Vec3_axisZ, axis_for_index(lockedAxis));
                xfA.q = b3MulQuat(c->rot1, alignment);
                xfB.q = b3MulQuat(c->rot2, alignment);
            }
        } else if (c->type == 3 && c->jointKind == 3) {
            int freeAxis = single_axis_index((~(c->lockedAxesMask >> 3)) & 0x7);
            b3Quat alignment = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisZ, axis_for_index(freeAxis));
            xfA.q = b3MulQuat(c->rot1, alignment);
            xfB.q = b3MulQuat(c->rot2, alignment);
        } else if (c->type == 3 && c->jointKind == 4) {
            int freeAxis = single_axis_index((~c->lockedAxesMask) & 0x7);
            b3Quat alignment = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisX, axis_for_index(freeAxis));
            xfA.q = b3MulQuat(c->rot1, alignment);
            xfB.q = b3MulQuat(c->rot2, alignment);
        }

        b3Joint_SetLocalFrameA(c->jointId, xfA);
        b3Joint_SetLocalFrameB(c->jointId, xfB);
        if (wakeBodies) b3Joint_WakeBodies(c->jointId);
    } else if (c->type == 1) { // Revolute
        b3Transform xfA, xfB;
        xfA.p = c->pos1;
        xfA.q = c->rot1;
        xfB.p = c->pos2;
        xfB.q = c->rot2;

        b3Joint_SetLocalFrameA(c->jointId, xfA);
        b3Joint_SetLocalFrameB(c->jointId, xfB);
        if (wakeBodies) b3Joint_WakeBodies(c->jointId);
    }
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_setConstraintFrame(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jlong handle, jint side,
    jdouble lx, jdouble ly, jdouble lz, jdouble qx, jdouble qy, jdouble qz, jdouble qw)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    Vox3DConstraint* c = find_constraint(scene, handle);
    if (c && c->valid) {
        if (side == 0) {
            c->sourcePos1[0] = lx;
            c->sourcePos1[1] = ly;
            c->sourcePos1[2] = lz;
            c->rot1 = make_safe_quat(qx, qy, qz, qw);
            move_constraint_world_anchor(c, false);
        } else {
            c->sourcePos2[0] = lx;
            c->sourcePos2[1] = ly;
            c->sourcePos2[2] = lz;
            c->rot2 = make_safe_quat(qx, qy, qz, qw);
            move_constraint_world_anchor(c, true);
        }
        update_joint_frames(scene, c, true);
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_setConstraintContactsEnabled(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jlong handle, jboolean enabled)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    Vox3DConstraint* c = find_constraint(scene, handle);
    if (c && c->valid) {
        c->contactsEnabled = enabled ? true : false;
        sync_constraint_contact_filter(scene, c);
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_getConstraintImpulses(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jlong handle, jdoubleArray storeArray)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    jdouble store[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    VOX_MUTEX_LOCK(scene->mutex);
    Vox3DConstraint* c = find_constraint(scene, handle);
    if (c) {
        for (int i = 0; i < 6; i++) store[i] = c->impulses[i];
        if (b3Joint_IsValid(c->jointId)) {
            b3Vec3 force = b3Joint_GetConstraintForce(c->jointId);
            b3Vec3 torque = b3Joint_GetConstraintTorque(c->jointId);
            // Box3D reports the final substep impulse as force/torque. Scale it
            // to Sable's full physics step, whose API exposes solver impulses.
            double fullStep = scene->lastTimeStep;
            store[0] += force.x * fullStep;
            store[1] += force.y * fullStep;
            store[2] += force.z * fullStep;
            store[3] += torque.x * fullStep;
            store[4] += torque.y * fullStep;
            store[5] += torque.z * fullStep;
        }
    }
    VOX_MUTEX_UNLOCK(scene->mutex);

    (*env)->SetDoubleArrayRegion(env, storeArray, 0, 6, store);
}

JNIEXPORT jboolean JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_isConstraintValid(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jlong handle)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return JNI_FALSE;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    Vox3DConstraint* c = find_constraint(scene, handle);
    if (c && !constraint_backend_valid(scene, c)) {
        destroy_constraint_resources(c, false);
        c->valid = false;
        c = NULL;
    }
    jboolean valid = c ? JNI_TRUE : JNI_FALSE;
    VOX_MUTEX_UNLOCK(scene->mutex);
    return valid;
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_removeConstraint(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jlong handle)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    Vox3DConstraint* c = find_constraint(scene, handle);
    if (c) {
        destroy_constraint_resources(c, true);
        c->valid = false;
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_setConstraintMotor(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jlong handle, jint axis,
    jdouble desiredPosition, jdouble stiffness, jdouble damping, jboolean hasForceLimit, jdouble maxForce)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    Vox3DConstraint* c = find_constraint(scene, handle);
    if (c && c->valid) {
        if (c->type == 1 && axis == 3) { // RotaryConstraint.DEFAULT_AXIS (ANGULAR_X)
            c->motorTarget[3] = isfinite(desiredPosition) ? desiredPosition : 0.0;
            c->motorStiffness[3] = isfinite(stiffness) ? fmax(0.0, stiffness) : 0.0;
            c->motorDamping[3] = isfinite(damping) ? fmax(0.0, damping) : 0.0;
            c->motorHasForceLimit[3] = hasForceLimit ? true : false;
            c->motorMaxForce[3] = isfinite(maxForce) ? fmax(0.0, maxForce) : 0.0;
            c->motorConfigured[3] = true;
            if (b3Joint_IsValid(c->jointId)) {
                // The Sable API specifies direct P/D coefficients. Box3D's
                // revolute spring uses hertz/damping-ratio values, so apply the
                // motor ourselves and leave Box3D responsible for hinge locks.
                b3RevoluteJoint_EnableSpring(c->jointId, false);
                b3Joint_WakeBodies(c->jointId);
            }
        } else if (c->type == 2) { // FreeConstraint
            if (axis >= 0 && axis < 6) {
                c->motorTarget[axis] = isfinite(desiredPosition) ? desiredPosition : 0.0;
                c->motorStiffness[axis] = isfinite(stiffness) ? fmax(0.0, stiffness) : 0.0;
                c->motorDamping[axis] = isfinite(damping) ? fmax(0.0, damping) : 0.0;
                c->motorHasForceLimit[axis] = hasForceLimit ? true : false;
                c->motorMaxForce[axis] = isfinite(maxForce) ? fmax(0.0, maxForce) : 0.0;
                c->motorConfigured[axis] = true;

                b3BodyId bodyA = get_constraint_body(scene, c, false);
                b3BodyId bodyB = get_constraint_body(scene, c, true);
                if (b3Body_IsValid(bodyA) && b3Body_GetType(bodyA) == b3_dynamicBody) {
                    b3Body_SetAwake(bodyA, true);
                }
                if (b3Body_IsValid(bodyB) && b3Body_GetType(bodyB) == b3_dynamicBody) {
                    b3Body_SetAwake(bodyB, true);
                }
            }
        } else if (c->type == 0) { // Fixed
            // Rapier masks motors off on every locked axis. A fixed constraint
            // locks all six, so its motor setters are intentionally no-ops.
        } else if (c->type == 3 && axis >= 0 && axis < 6) { // Generic
            c->motorTarget[axis] = isfinite(desiredPosition) ? desiredPosition : 0.0;
            c->motorStiffness[axis] = isfinite(stiffness) ? fmax(0.0, stiffness) : 0.0;
            c->motorDamping[axis] = isfinite(damping) ? fmax(0.0, damping) : 0.0;
            c->motorHasForceLimit[axis] = hasForceLimit ? true : false;
            c->motorMaxForce[axis] = isfinite(maxForce) ? fmax(0.0, maxForce) : 0.0;
            c->motorConfigured[axis] = true;

            if (b3Joint_IsValid(c->jointId)) {
                if (c->jointKind == 2) b3SphericalJoint_EnableSpring(c->jointId, false);
                if (c->jointKind == 3) b3RevoluteJoint_EnableSpring(c->jointId, false);
                if (c->jointKind == 4) b3PrismaticJoint_EnableSpring(c->jointId, false);
                b3Joint_WakeBodies(c->jointId);
            } else {
                b3BodyId bodyA = get_constraint_body(scene, c, false);
                b3BodyId bodyB = get_constraint_body(scene, c, true);
                if (b3Body_IsValid(bodyA) && b3Body_GetType(bodyA) == b3_dynamicBody) {
                    b3Body_SetAwake(bodyA, true);
                }
                if (b3Body_IsValid(bodyB) && b3Body_GetType(bodyB) == b3_dynamicBody) {
                    b3Body_SetAwake(bodyB, true);
                }
            }
        }
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_setConstraintLimit(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jlong handle, jint axis, jdouble min, jdouble max)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    Vox3DConstraint* c = find_constraint(scene, handle);
    if (c && b3Joint_IsValid(c->jointId)) {
        if (c->type == 1 && axis == 3) { // RotaryConstraint.DEFAULT_AXIS (ANGULAR_X)
            b3RevoluteJoint_EnableLimit(c->jointId, true);
            b3RevoluteJoint_SetLimits(c->jointId, (float)min, (float)max);
        } else if (c->type == 3 && c->jointKind == 3 && axis >= 3) {
            int freeAxis = single_axis_index((~(c->lockedAxesMask >> 3)) & 0x7);
            if (axis - 3 == freeAxis) {
                b3RevoluteJoint_EnableLimit(c->jointId, true);
                b3RevoluteJoint_SetLimits(c->jointId, (float)min, (float)max);
            }
        } else if (c->type == 3 && c->jointKind == 4 && axis < 3) {
            int freeAxis = single_axis_index((~c->lockedAxesMask) & 0x7);
            if (axis == freeAxis) {
                b3PrismaticJoint_EnableLimit(c->jointId, true);
                b3PrismaticJoint_SetLimits(c->jointId, (float)min, (float)max);
            }
        }
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT jboolean JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_lockConstraintAxes(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jlong handle, jbyte mask)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return JNI_FALSE;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;
    VOX_MUTEX_LOCK(scene->mutex);
    Vox3DConstraint* c = find_constraint(scene, handle);
    if (c && c->valid && c->type == 3) {
        int newMask = c->lockedAxesMask | (((int)mask) & 0x3F);
        if (newMask == c->lockedAxesMask) {
            VOX_MUTEX_UNLOCK(scene->mutex);
            return JNI_TRUE;
        }
        if (!generic_mask_supported(newMask)) {
            VOX_MUTEX_UNLOCK(scene->mutex);
            return JNI_FALSE;
        }
        c->lockedAxesMask = newMask;
        rebuild_generic_joint(scene, c);
        VOX_MUTEX_UNLOCK(scene->mutex);
        return JNI_TRUE;
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
    return JNI_FALSE;
}

// -------------------------------------------------------------------------
// Kinematic Contraptions
// -------------------------------------------------------------------------

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_createKinematicContraption(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint mountId, jint id, jdoubleArray poseArray)
{
    (void)clazz;
    if (sceneHandle == 0 || poseArray == NULL || (*env)->GetArrayLength(env, poseArray) < 7) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    jdouble pose[7];
    (*env)->GetDoubleArrayRegion(env, poseArray, 0, 7, pose);
    if (!isfinite(pose[0]) || !isfinite(pose[1]) || !isfinite(pose[2])) return;
    b3Quat rotation = make_safe_quat(pose[3], pose[4], pose[5], pose[6]);

    VOX_MUTEX_LOCK(scene->mutex);

    KinematicContraption* existing = find_contraption(scene, id);
    if (existing) release_contraption(scene, existing, true, false);

    KinematicContraption* con = alloc_contraption(scene, id);
    if (con) {
        con->mountId = mountId;
        con->posePosition[0] = pose[0];
        con->posePosition[1] = pose[1];
        con->posePosition[2] = pose[2];
        con->poseRotation = rotation;
        con->centerOfMass[0] = 0.0;
        con->centerOfMass[1] = 0.0;
        con->centerOfMass[2] = 0.0;

        if (mountId == -1) {
            b3BodyDef bodyDef = b3DefaultBodyDef();
            bodyDef.type = b3_kinematicBody;
            bodyDef.position = (b3Pos){ pose[0], pose[1], pose[2] };
            bodyDef.rotation = rotation;
            con->bodyId = b3CreateBody(scene->worldId, &bodyDef);
            con->ownsBody = true;
        } else {
            SubLevelBody* mount = find_sublevel(scene, mountId);
            if (!mount || !b3Body_IsValid(mount->bodyId)) {
                con->valid = false;
            } else {
                // The hulls are created directly on the dynamic parent. This
                // makes contacts affect the ship instead of an independent
                // infinite-mass kinematic body.
                con->bodyId = mount->bodyId;
                con->ownsBody = false;
                // createKinematicContraption receives the logical position;
                // subsequent updates are already COM-relative on the Java
                // side. Normalize the initial pose here as well so a perfectly
                // stationary contraption is correct even when no first update
                // crosses the upload threshold.
                con->posePosition[0] -= mount->centerOfMass[0];
                con->posePosition[1] -= mount->centerOfMass[1];
                con->posePosition[2] -= mount->centerOfMass[2];
            }
        }
    }

    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_removeKinematicContraption(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint id)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    KinematicContraption* con = find_contraption(scene, id);
    if (con) {
        release_contraption(scene, con, true, false);
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_setKinematicContraptionTransform(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint id,
    jdoubleArray comArray, jdoubleArray poseArray, jdoubleArray velArray)
{
    (void)clazz;
    if (sceneHandle == 0 || comArray == NULL || poseArray == NULL || velArray == NULL ||
        (*env)->GetArrayLength(env, comArray) < 3 ||
        (*env)->GetArrayLength(env, poseArray) < 7 ||
        (*env)->GetArrayLength(env, velArray) < 6) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    jdouble com[3];
    (*env)->GetDoubleArrayRegion(env, comArray, 0, 3, com);
    jdouble pose[7];
    (*env)->GetDoubleArrayRegion(env, poseArray, 0, 7, pose);
    jdouble vel[6];
    (*env)->GetDoubleArrayRegion(env, velArray, 0, 6, vel);

    for (int i = 0; i < 3; i++) {
        if (!isfinite(com[i]) || !isfinite(pose[i]) || !isfinite(vel[i]) || !isfinite(vel[i + 3])) {
            return;
        }
    }
    b3Quat rotation = make_safe_quat(pose[3], pose[4], pose[5], pose[6]);

    VOX_MUTEX_LOCK(scene->mutex);
    KinematicContraption* con = find_contraption(scene, id);
    if (con) {
        bool centerChanged =
            con->centerOfMass[0] != com[0] ||
            con->centerOfMass[1] != com[1] ||
            con->centerOfMass[2] != com[2];
        con->centerOfMass[0] = com[0];
        con->centerOfMass[1] = com[1];
        con->centerOfMass[2] = com[2];
        con->posePosition[0] = pose[0];
        con->posePosition[1] = pose[1];
        con->posePosition[2] = pose[2];
        con->poseRotation = rotation;
        for (int i = 0; i < 3; i++) {
            con->linearVelocity[i] = vel[i];
            con->angularVelocity[i] = vel[i + 3];
        }

        if (!con->ownsBody) {
            SubLevelBody* mount = find_sublevel(scene, con->mountId);
            if (!mount || !b3Body_IsValid(mount->bodyId) ||
                !B3_ID_EQUALS(mount->bodyId, con->bodyId)) {
                release_contraption(scene, con, true, true);
            } else {
                // A mounted collider's pose is relative to its parent, so its
                // baked hull transforms must change whenever pose or COM does.
                rebuild_contraption_shapes(con);
            }
        } else if (b3Body_IsValid(con->bodyId)) {
            // The body is advanced to posePosition/poseRotation immediately
            // before the next world step via b3Body_SetTargetTransform.
            if (centerChanged) rebuild_contraption_shapes(con);
        }
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_addKinematicContraptionChunkSection(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint id, jint x, jint y, jint z, jintArray dataArray)
{
    (void)clazz;
    if (sceneHandle == 0 || dataArray == NULL || (*env)->GetArrayLength(env, dataArray) < 4096) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    jint data[4096];
    (*env)->GetIntArrayRegion(env, dataArray, 0, 4096, data);

    VOX_MUTEX_LOCK(scene->mutex);
    KinematicContraption* con = find_contraption(scene, id);
    if (con && b3Body_IsValid(con->bodyId)) {
        size_t additionalShapeCount = 0;
        for (int idx = 0; idx < 4096; idx++) {
            int colliderId = (int)(((uint32_t)data[idx] >> 16) & 0xFFFFu);
            if (colliderId <= 0 || colliderId > g_voxelColliderCount) continue;
            VoxelColliderDef* def = &g_voxelColliders[colliderId - 1];
            if (def->isFluid || def->boxCount <= 0) continue;
            if (additionalShapeCount > (size_t)INT_MAX - (size_t)def->boxCount) {
                additionalShapeCount = (size_t)INT_MAX + 1u;
                break;
            }
            additionalShapeCount += (size_t)def->boxCount;
        }

        if (additionalShapeCount > (size_t)(INT_MAX - con->shapeCount) ||
            !ensure_contraption_shape_capacity(
                con, con->shapeCount + (int)additionalShapeCount)) {
            VOX_MUTEX_UNLOCK(scene->mutex);
            return;
        }

        for (int bx = 0; bx < 16; bx++) {
            for (int bz = 0; bz < 16; bz++) {
                for (int by = 0; by < 16; by++) {
                    int idx = bx + (bz << 4) + (by << 8);
                    int packed = data[idx];
                    int colliderId = (int)(((uint32_t)packed >> 16) & 0xFFFFu);
                    if (colliderId <= 0 || colliderId > g_voxelColliderCount) continue;

                    VoxelColliderDef* def = &g_voxelColliders[colliderId - 1];
                    if (def->isFluid || def->boxCount == 0) continue;

                    int64_t wx64 = (int64_t)x * 16 + bx;
                    int64_t wy64 = (int64_t)y * 16 + by;
                    int64_t wz64 = (int64_t)z * 16 + bz;
                    if (wx64 < INT_MIN || wx64 > INT_MAX ||
                        wy64 < INT_MIN || wy64 > INT_MAX ||
                        wz64 < INT_MIN || wz64 > INT_MAX) continue;

                    for (int bi = 0; bi < def->boxCount; bi++) {
                        VoxelBox* vb = &def->boxes[bi];
                        float hx = (float)((vb->max[0] - vb->min[0]) * 0.5);
                        float hy = (float)((vb->max[1] - vb->min[1]) * 0.5);
                        float hz = (float)((vb->max[2] - vb->min[2]) * 0.5);
                        if (hx <= 0.0001f || hy <= 0.0001f || hz <= 0.0001f) continue;

                        KinematicContraptionShape* shape = &con->shapes[con->shapeCount++];
                        memset(shape, 0, sizeof(*shape));
                        shape->shapeId = b3_nullShapeId;
                        shape->contraptionId = con->id;
                        shape->sourceCenter[0] = (double)wx64 + vb->min[0] + (double)hx;
                        shape->sourceCenter[1] = (double)wy64 + vb->min[1] + (double)hy;
                        shape->sourceCenter[2] = (double)wz64 + vb->min[2] + (double)hz;
                        shape->halfExtents[0] = hx;
                        shape->halfExtents[1] = hy;
                        shape->halfExtents[2] = hz;
                        shape->blockPosition[0] = (int)wx64;
                        shape->blockPosition[1] = (int)wy64;
                        shape->blockPosition[2] = (int)wz64;
                        shape->colliderId = colliderId;
                        shape->shapeId = create_contraption_shape(con, shape);
                    }
                }
            }
        }
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
}

// -------------------------------------------------------------------------
// Ropes
// -------------------------------------------------------------------------

static int64_t g_nextRopeId = 1;

static b3BodyId get_rope_mount_body(Vox3DScene* scene, const Vox3DRope* rope, bool end) {
    int subLevelId = end ? rope->endSubLevelId : rope->startSubLevelId;
    if (subLevelId != -1) return get_body_by_id(scene, subLevelId);

    b3BodyId anchorBody = end ? rope->endWorldAnchorBody : rope->startWorldAnchorBody;
    return b3Body_IsValid(anchorBody) ? anchorBody : b3_nullBodyId;
}

static b3Vec3 get_rope_mount_anchor(Vox3DScene* scene, const Vox3DRope* rope, bool end) {
    int subLevelId = end ? rope->endSubLevelId : rope->startSubLevelId;
    if (subLevelId == -1) return b3Vec3_zero;

    const double* location = end ? rope->endLocation : rope->startLocation;
    return get_local_anchor(scene, subLevelId, location[0], location[1], location[2]);
}

static void destroy_rope_world_anchor(Vox3DRope* rope, bool end) {
    b3BodyId* anchorBody = end ? &rope->endWorldAnchorBody : &rope->startWorldAnchorBody;
    if (b3Body_IsValid(*anchorBody)) b3DestroyBody(*anchorBody);
    *anchorBody = b3_nullBodyId;
}

JNIEXPORT jlong JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_createRope(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jdouble pointRadius,
    jdouble firstJointLength, jdoubleArray pointsArray, jint pointCount)
{
    (void)clazz;
    if (sceneHandle == 0) return 0;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    if (pointsArray == NULL || pointCount <= 0 || pointCount > INT_MAX / 3 ||
        (*env)->GetArrayLength(env, pointsArray) < pointCount * 3) {
        return 0;
    }

    jdouble* pts = (jdouble*)malloc((size_t)pointCount * 3 * sizeof(jdouble));
    if (!pts) return 0;
    (*env)->GetDoubleArrayRegion(env, pointsArray, 0, pointCount * 3, pts);

    VOX_MUTEX_LOCK(scene->mutex);
    int64_t rId = g_nextRopeId++;
    Vox3DRope* rope = alloc_rope(scene, rId);
    if (!rope) {
        free(pts);
        VOX_MUTEX_UNLOCK(scene->mutex);
        return 0;
    }
    if (!ensure_rope_capacity(rope, pointCount)) {
        rope->valid = false;
        free(pts);
        VOX_MUTEX_UNLOCK(scene->mutex);
        return 0;
    }

    rope->pointRadius = pointRadius;
    rope->firstJointLength = firstJointLength;
    rope->pointCount = pointCount;
    rope->hasStartAttachment = false;
    rope->hasEndAttachment = false;
    rope->startSubLevelId = -1;
    rope->endSubLevelId = -1;

    float r = (float)pointRadius;
    if (r < 0.01f) r = 0.05f;

    for (int i = 0; i < pointCount; i++) {
        rope->points[i] = (b3Pos){ pts[i * 3 + 0], pts[i * 3 + 1], pts[i * 3 + 2] };

        b3BodyDef bd = b3DefaultBodyDef();
        bd.type = b3_dynamicBody;
        bd.position = (b3Pos){ pts[i * 3 + 0], pts[i * 3 + 1], pts[i * 3 + 2] };
        bd.linearDamping = (float)scene->universalDrag;
        bd.angularDamping = (float)scene->universalDrag;
        bd.motionLocks.angularX = true;
        bd.motionLocks.angularY = true;
        bd.motionLocks.angularZ = true;
        rope->segmentBodies[i] = b3CreateBody(scene->worldId, &bd);

        b3ShapeDef sd = b3DefaultShapeDef();
        sd.density = 0.35f / (4.18879f * r * r * r);
        sd.baseMaterial.friction = 0.15f;
        sd.filter.categoryBits = (1ULL << 1);
        sd.filter.maskBits = (1ULL << 0);
        b3Sphere sphere = { b3Vec3_zero, r };
        b3CreateSphereShape(rope->segmentBodies[i], &sd, &sphere);

        if (i > 0) {
            b3DistanceJointDef dDef = b3DefaultDistanceJointDef();
            dDef.base.bodyIdA = rope->segmentBodies[i - 1];
            dDef.base.bodyIdB = rope->segmentBodies[i];
            dDef.base.localFrameA.p = b3Vec3_zero;
            dDef.base.localFrameA.q = b3Quat_identity;
            dDef.base.localFrameB.p = b3Vec3_zero;
            dDef.base.localFrameB.q = b3Quat_identity;
            // Rapier uses the configurable first segment length and one block
            // for every later rope segment, independent of a temporarily
            // stretched serialized pose.
            dDef.length = (i == 1) ? (float)firstJointLength : 1.0f;
            dDef.minLength = 0.0f;
            dDef.maxLength = dDef.length;
            dDef.enableLimit = true;
            dDef.enableSpring = true;
            dDef.hertz = 0.0f;
            dDef.dampingRatio = 1.0f;
            dDef.base.collideConnected = false;
            rope->joints[i - 1] = b3CreateDistanceJoint(scene->worldId, &dDef);
        }
    }

    free(pts);
    VOX_MUTEX_UNLOCK(scene->mutex);
    return rId;
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_removeRope(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jlong ropeId)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    Vox3DRope* rope = find_rope(scene, ropeId);
    if (rope) {
        if (rope->hasStartAttachment && b3Joint_IsValid(rope->startJoint)) {
            b3DestroyJoint(rope->startJoint, true);
            rope->hasStartAttachment = false;
        }
        if (rope->hasEndAttachment && b3Joint_IsValid(rope->endJoint)) {
            b3DestroyJoint(rope->endJoint, true);
            rope->hasEndAttachment = false;
        }
        destroy_rope_world_anchor(rope, false);
        destroy_rope_world_anchor(rope, true);
        for (int i = 0; i < rope->pointCount - 1; i++) {
            if (b3Joint_IsValid(rope->joints[i])) {
                b3DestroyJoint(rope->joints[i], true);
            }
        }
        for (int i = 0; i < rope->pointCount; i++) {
            if (b3Body_IsValid(rope->segmentBodies[i])) {
                b3DestroyBody(rope->segmentBodies[i]);
            }
        }
        free(rope->points);
        free(rope->segmentBodies);
        free(rope->joints);
        rope->points = NULL;
        rope->segmentBodies = NULL;
        rope->joints = NULL;
        rope->pointCount = 0;
        rope->pointCapacity = 0;
        rope->valid = false;
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_setRopeAttachment(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jlong ropeId, jint subLevelId,
    jdouble x, jdouble y, jdouble z, jboolean end)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    Vox3DRope* rope = find_rope(scene, ropeId);
    if (rope && rope->pointCount > 0) {
        int idx = end ? (rope->pointCount - 1) : 0;
        b3BodyId segBody = rope->segmentBodies[idx];
        if (b3Body_IsValid(segBody)) {
            if (end) {
                if (rope->hasEndAttachment && b3Joint_IsValid(rope->endJoint)) {
                    b3DestroyJoint(rope->endJoint, true);
                }
                rope->hasEndAttachment = false;
                rope->endJoint = b3_nullJointId;
                destroy_rope_world_anchor(rope, true);
            } else {
                if (rope->hasStartAttachment && b3Joint_IsValid(rope->startJoint)) {
                    b3DestroyJoint(rope->startJoint, true);
                }
                rope->hasStartAttachment = false;
                rope->startJoint = b3_nullJointId;
                destroy_rope_world_anchor(rope, false);
            }

            b3BodyId mountBody = subLevelId == -1
                ? create_world_anchor_body(scene, x, y, z)
                : get_body_by_id(scene, subLevelId);
            if (b3Body_IsValid(mountBody)) {
                b3DistanceJointDef dDef = b3DefaultDistanceJointDef();
                dDef.base.bodyIdA = mountBody;
                dDef.base.bodyIdB = segBody;
                dDef.base.localFrameA.p = subLevelId == -1
                    ? b3Vec3_zero
                    : get_local_anchor(scene, subLevelId, x, y, z);
                dDef.base.localFrameA.q = b3Quat_identity;
                dDef.base.localFrameB.p = b3Vec3_zero;
                dDef.base.localFrameB.q = b3Quat_identity;
                dDef.length = 0.05f;
                dDef.minLength = 0.0f;
                dDef.maxLength = 0.05f;
                dDef.enableLimit = true;
                dDef.enableSpring = true;
                dDef.hertz = 0.0f;
                dDef.dampingRatio = 1.0f;
                dDef.base.collideConnected = false;

                b3JointId jId = b3CreateDistanceJoint(scene->worldId, &dDef);
                if (b3Joint_IsValid(jId)) {
                    b3Joint_WakeBodies(jId);
                    b3Body_SetAwake(mountBody, true);
                    b3Body_SetAwake(segBody, true);

                    if (end) {
                        rope->endJoint = jId;
                        rope->endWorldAnchorBody = subLevelId == -1 ? mountBody : b3_nullBodyId;
                        rope->hasEndAttachment = true;
                        rope->endSubLevelId = subLevelId;
                        rope->endLocation[0] = x;
                        rope->endLocation[1] = y;
                        rope->endLocation[2] = z;
                    } else {
                        rope->startJoint = jId;
                        rope->startWorldAnchorBody = subLevelId == -1 ? mountBody : b3_nullBodyId;
                        rope->hasStartAttachment = true;
                        rope->startSubLevelId = subLevelId;
                        rope->startLocation[0] = x;
                        rope->startLocation[1] = y;
                        rope->startLocation[2] = z;
                    }
                } else if (subLevelId == -1) {
                    b3DestroyBody(mountBody);
                }
            }
        }
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_addRopePointAtStart(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jlong ropeId, jdouble x, jdouble y, jdouble z)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    Vox3DRope* rope = find_rope(scene, ropeId);
    if (rope && ensure_rope_capacity(rope, rope->pointCount + 1)) {
        if (rope->pointCount > 1 && b3Joint_IsValid(rope->joints[0])) {
            b3DistanceJoint_SetLength(rope->joints[0], 1.0f);
            b3DistanceJoint_SetLengthRange(rope->joints[0], 0.0f, 1.0f);
        }

        for (int i = rope->pointCount; i > 0; i--) {
            rope->points[i] = rope->points[i - 1];
            rope->segmentBodies[i] = rope->segmentBodies[i - 1];
        }
        for (int i = rope->pointCount - 1; i > 0; i--) {
            rope->joints[i] = rope->joints[i - 1];
        }

        rope->points[0] = (b3Pos){ x, y, z };
        b3BodyDef bd = b3DefaultBodyDef();
        bd.type = b3_dynamicBody;
        bd.position = (b3Pos){ x, y, z };
        bd.linearDamping = (float)scene->universalDrag;
        bd.angularDamping = (float)scene->universalDrag;
        bd.motionLocks.angularX = true;
        bd.motionLocks.angularY = true;
        bd.motionLocks.angularZ = true;
        rope->segmentBodies[0] = b3CreateBody(scene->worldId, &bd);

        float r = (float)rope->pointRadius;
        if (r < 0.01f) r = 0.05f;
        b3ShapeDef sd = b3DefaultShapeDef();
        sd.density = 0.35f / (4.18879f * r * r * r);
        sd.baseMaterial.friction = 0.15f;
        sd.filter.categoryBits = (1ULL << 1);
        sd.filter.maskBits = (1ULL << 0);
        b3Sphere sphere = { b3Vec3_zero, r };
        b3CreateSphereShape(rope->segmentBodies[0], &sd, &sphere);

        if (rope->pointCount > 0) {
            b3DistanceJointDef dDef = b3DefaultDistanceJointDef();
            dDef.base.bodyIdA = rope->segmentBodies[0];
            dDef.base.bodyIdB = rope->segmentBodies[1];
            dDef.base.localFrameA.p = b3Vec3_zero;
            dDef.base.localFrameA.q = b3Quat_identity;
            dDef.base.localFrameB.p = b3Vec3_zero;
            dDef.base.localFrameB.q = b3Quat_identity;
            dDef.length = (float)rope->firstJointLength;
            dDef.minLength = 0.0f;
            dDef.maxLength = (float)rope->firstJointLength;
            dDef.enableSpring = true;
            dDef.hertz = 0.0f;
            dDef.dampingRatio = 1.0f;
            dDef.enableLimit = true;
            dDef.base.collideConnected = false;
            rope->joints[0] = b3CreateDistanceJoint(scene->worldId, &dDef);
        }

        rope->pointCount++;

        if (rope->hasStartAttachment) {
            if (b3Joint_IsValid(rope->startJoint)) {
                b3DestroyJoint(rope->startJoint, true);
            }
            b3BodyId mountBody = get_rope_mount_body(scene, rope, false);
            if (b3Body_IsValid(mountBody)) {
                b3DistanceJointDef aDef = b3DefaultDistanceJointDef();
                aDef.base.bodyIdA = mountBody;
                aDef.base.bodyIdB = rope->segmentBodies[0];
                aDef.base.localFrameA.p = get_rope_mount_anchor(scene, rope, false);
                aDef.base.localFrameA.q = b3Quat_identity;
                aDef.base.localFrameB.p = b3Vec3_zero;
                aDef.base.localFrameB.q = b3Quat_identity;
                aDef.length = 0.05f;
                aDef.minLength = 0.0f;
                aDef.maxLength = 0.05f;
                aDef.enableSpring = true;
                aDef.hertz = 0.0f;
                aDef.dampingRatio = 1.0f;
                aDef.enableLimit = true;
                aDef.base.collideConnected = false;
                rope->startJoint = b3CreateDistanceJoint(scene->worldId, &aDef);
                if (b3Joint_IsValid(rope->startJoint)) {
                    b3Joint_WakeBodies(rope->startJoint);
                } else {
                    rope->startJoint = b3_nullJointId;
                    rope->hasStartAttachment = false;
                    destroy_rope_world_anchor(rope, false);
                }
            }
        }
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_removeRopePointAtStart(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jlong ropeId)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    Vox3DRope* rope = find_rope(scene, ropeId);
    if (rope && rope->pointCount > 1) {
        if (rope->hasStartAttachment && b3Joint_IsValid(rope->startJoint)) {
            b3DestroyJoint(rope->startJoint, true);
        }
        if (b3Joint_IsValid(rope->joints[0])) {
            b3DestroyJoint(rope->joints[0], true);
        }
        if (b3Body_IsValid(rope->segmentBodies[0])) {
            b3DestroyBody(rope->segmentBodies[0]);
        }

        for (int i = 0; i < rope->pointCount - 1; i++) {
            rope->points[i] = rope->points[i + 1];
            rope->segmentBodies[i] = rope->segmentBodies[i + 1];
        }
        for (int i = 0; i < rope->pointCount - 2; i++) {
            rope->joints[i] = rope->joints[i + 1];
        }
        rope->pointCount--;

        if (rope->pointCount > 1 && b3Joint_IsValid(rope->joints[0])) {
            b3DistanceJoint_SetLength(rope->joints[0], (float)rope->firstJointLength);
            b3DistanceJoint_SetLengthRange(rope->joints[0], 0.0f, (float)rope->firstJointLength);
        }

        if (rope->hasStartAttachment) {
            b3BodyId mountBody = get_rope_mount_body(scene, rope, false);
            if (b3Body_IsValid(mountBody)) {
                b3DistanceJointDef aDef = b3DefaultDistanceJointDef();
                aDef.base.bodyIdA = mountBody;
                aDef.base.bodyIdB = rope->segmentBodies[0];
                aDef.base.localFrameA.p = get_rope_mount_anchor(scene, rope, false);
                aDef.base.localFrameA.q = b3Quat_identity;
                aDef.base.localFrameB.p = b3Vec3_zero;
                aDef.base.localFrameB.q = b3Quat_identity;
                aDef.length = 0.05f;
                aDef.minLength = 0.0f;
                aDef.maxLength = 0.05f;
                aDef.enableSpring = true;
                aDef.hertz = 0.0f;
                aDef.dampingRatio = 1.0f;
                aDef.enableLimit = true;
                aDef.base.collideConnected = false;
                rope->startJoint = b3CreateDistanceJoint(scene->worldId, &aDef);
                if (b3Joint_IsValid(rope->startJoint)) {
                    b3Joint_WakeBodies(rope->startJoint);
                } else {
                    rope->startJoint = b3_nullJointId;
                    rope->hasStartAttachment = false;
                    destroy_rope_world_anchor(rope, false);
                }
            }
        }
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_wakeUpRope(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jlong ropeId)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    Vox3DRope* rope = find_rope(scene, ropeId);
    if (rope) {
        for (int i = 0; i < rope->pointCount; i++) {
            if (b3Body_IsValid(rope->segmentBodies[i])) {
                b3Body_SetAwake(rope->segmentBodies[i], true);
            }
        }
        if (rope->hasStartAttachment && b3Joint_IsValid(rope->startJoint)) {
            b3Joint_WakeBodies(rope->startJoint);
        }
        if (rope->hasEndAttachment && b3Joint_IsValid(rope->endJoint)) {
            b3Joint_WakeBodies(rope->endJoint);
        }
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_setRopeFirstSegmentLength(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jlong ropeId, jdouble firstSegmentLength)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    Vox3DRope* rope = find_rope(scene, ropeId);
    if (rope) {
        rope->firstJointLength = firstSegmentLength;
        if (rope->pointCount > 1 && b3Joint_IsValid(rope->joints[0])) {
            b3DistanceJoint_SetLength(rope->joints[0], (float)firstSegmentLength);
            b3DistanceJoint_SetLengthRange(rope->joints[0], 0.0f, (float)firstSegmentLength);
        }
    }
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT jdoubleArray JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_queryRope(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jlong ropeId)
{
    (void)clazz;
    if (sceneHandle == 0) return (*env)->NewDoubleArray(env, 0);
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    Vox3DRope* rope = find_rope(scene, ropeId);
    if (!rope || rope->pointCount == 0) {
        VOX_MUTEX_UNLOCK(scene->mutex);
        return (*env)->NewDoubleArray(env, 0);
    }

    int count = rope->pointCount;
    jdouble* data = (jdouble*)malloc((size_t)count * 3 * sizeof(jdouble));
    if (!data) {
        VOX_MUTEX_UNLOCK(scene->mutex);
        return (*env)->NewDoubleArray(env, 0);
    }
    for (int i = 0; i < count; i++) {
        if (b3Body_IsValid(rope->segmentBodies[i])) {
            b3WorldTransform xf = b3Body_GetTransform(rope->segmentBodies[i]);
            data[i * 3 + 0] = xf.p.x;
            data[i * 3 + 1] = xf.p.y;
            data[i * 3 + 2] = xf.p.z;
        } else {
            data[i * 3 + 0] = rope->points[i].x;
            data[i * 3 + 1] = rope->points[i].y;
            data[i * 3 + 2] = rope->points[i].z;
        }
    }
    VOX_MUTEX_UNLOCK(scene->mutex);

    jdoubleArray arr = (*env)->NewDoubleArray(env, count * 3);
    (*env)->SetDoubleArrayRegion(env, arr, 0, count * 3, data);
    free(data);
    return arr;
}

// -------------------------------------------------------------------------
// Configuration
// -------------------------------------------------------------------------

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_configFrequencyAndDamping(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jdouble frequency, jdouble damping)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0 || !isfinite(frequency) || !isfinite(damping)) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;
    VOX_MUTEX_LOCK(scene->mutex);
    b3World_SetContactTuning(scene->worldId, (float)fmax(0.0, frequency),
                             (float)fmax(0.0, damping), scene->contactSpeed);
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_configSolverIterations(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint solverIterations, jint pgsIterations, jint stabilizationIterations)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;
    int totalIterations = solverIterations + pgsIterations + stabilizationIterations;
    int subSteps = (totalIterations + 5) / 6;
    if (subSteps < 1) subSteps = 1;
    if (subSteps > 16) subSteps = 16;
    VOX_MUTEX_LOCK(scene->mutex);
    scene->solverSubSteps = subSteps;
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_configMinIslandSize(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jint islandSize)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;
    VOX_MUTEX_LOCK(scene->mutex);
    scene->minIslandSize = islandSize > 0 ? islandSize : 1;
    VOX_MUTEX_UNLOCK(scene->mutex);
}

JNIEXPORT jint JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_nextBodyID(
    JNIEnv *env, jclass clazz)
{
    (void)env; (void)clazz;
    ensure_global_mutex();
    VOX_MUTEX_LOCK(g_globalMutex);
    int id = g_nextBodyId++;
    VOX_MUTEX_UNLOCK(g_globalMutex);
    return id;
}

JNIEXPORT void JNICALL Java_dev_ryanhcode_sable_physics_impl_vox3d_Vox3D_setWind(
    JNIEnv *env, jclass clazz, jlong sceneHandle, jdouble wx, jdouble wy, jdouble wz, jdouble drag, jdouble lift)
{
    (void)env; (void)clazz;
    if (sceneHandle == 0) return;
    Vox3DScene* scene = (Vox3DScene*)(uintptr_t)sceneHandle;

    VOX_MUTEX_LOCK(scene->mutex);
    scene->wind = (b3Vec3){ (float)wx, (float)wy, (float)wz };
    scene->aeroDrag = (float)drag;
    scene->aeroLift = (float)lift;
    VOX_MUTEX_UNLOCK(scene->mutex);
}
