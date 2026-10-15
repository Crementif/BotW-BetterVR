#pragma once

#pragma pack(push, 1)
namespace sead {
    struct SafeString : BETypeCompatible {
        BEType<uint32_t> c_str;
        BEType<uint32_t> vtable;
    };

    struct BufferedSafeString : SafeString {
        BEType<int32_t> length;
    };
    static_assert(sizeof(BufferedSafeString) == 0x0C, "BufferedSafeString size mismatch");

    struct FixedSafeString40 : BufferedSafeString {
        char data[0x40];

        std::string getLE() const {
            if (c_str.getLE() == 0) {
                return std::string();
            }
            return std::string(data, strnlen(data, sizeof(data)));
        }
    };
    static_assert(sizeof(FixedSafeString40) == 0x4C, "FixedSafeString40 size mismatch");

	struct FixedSafeString100 : BufferedSafeString {
        char data[0x100];

        std::string getLE() const {
            if (c_str.getLE() == 0) {
                return std::string();
            }
            return std::string(data, strnlen(data, sizeof(data)));
        }
    };
    static_assert(sizeof(FixedSafeString100) == 0x10C, "FixedSafeString100 size mismatch");

    struct PtrArrayImpl {
        BEType<uint32_t> size;
        BEType<uint32_t> capacity;
        BEType<uint32_t> data;
    };

    struct Buffer : BETypeCompatible {
        BEType<int32_t> size;
        BEType<uint32_t> data;
    };
    static_assert(sizeof(Buffer) == 0x08, "Buffer size mismatch");
};

struct ActorPhysics {
    BEType<uint32_t> __vftable;
    sead::SafeString actorName;
    BEType<uint32_t> *physicsParamSet;
    BEType<uint32_t> flags;
    BEType<uint32_t> dword14;
    BEType<uint32_t> dword18;
    BEType<float> scale;
    BEType<uint32_t> gsysModel;
    sead::PtrArrayImpl rigidBodies;
    sead::PtrArrayImpl collisionInfo;
    sead::PtrArrayImpl contactInfo;
};

struct BaseProc {
    BEType<uint32_t> secondVTable;
    sead::FixedSafeString40 name;
    BEType<uint32_t> id;
    BEType<uint8_t> state;
    BEType<uint8_t> prio;
    BEType<uint8_t> unk_52;
    BEType<uint8_t> unk_53;
    PADDED_BYTES(0x54, 0xE0);
    BEType<uint32_t> vtable;
};
static_assert(sizeof(BaseProc) == 0xEC, "BaseProc size mismatch");

enum ActorFlags : int32_t {
    ActorFlags_1 = 0x1,
    ActorFlags_PhysicsPauseDisable = 0x2,
    ActorFlags_SetPhysicsMtx = 0x4,
    ActorFlags_DisableUpdateMtxFromPhysics = 0x8,
    ActorFlags_UndispCut = 0x10,
    ActorFlags_ModelBind = 0x20,
    ActorFlags_ParamW = 0x40,
    ActorFlags_80 = 0x80,
    ActorFlags_100 = 0x100,
    ActorFlags_200 = 0x200,
    ActorFlags_400 = 0x400,
    ActorFlags_KeepLinkTagOrAreaAliveMaybe = 0x4000,
    ActorFlags_EnableForbidPushJob = 0x40000,
    ActorFlags_DisableForbidPushJob = 0x80000,
    ActorFlags_IsLinkTag_ComplexTag_EventTag_AreaManagement = 0x800000,
    ActorFlags_OnlyPushJobType4 = 0x1000000,
    ActorFlags_VillagerRegistered = 0x4000000,
    ActorFlags_CheckDeleteDistanceEveryFrame = 0x8000000,
    ActorFlags_ForceCalcInEvent = 0x10000000,
    ActorFlags_IsCameraOrEditCamera = 0x20000000,
    ActorFlags_InitializedMaybe = 0x40000000,
    ActorFlags_PrepareForDeleteMaybe = (int32_t)0x80000000,
};

enum ActorFlags2 : int32_t {
    ActorFlags2_1 = 0x1,
    ActorFlags2_2 = 0x2,
    ActorFlags2_4 = 0x4,
    ActorFlags2_InstEventFlag = 0x8,
    ActorFlags2_10 = 0x10,
    ActorFlags2_Invisible = 0x20,
    ActorFlags2_40 = 0x40,
    ActorFlags2_NoDistanceCheck = 0x80,
    ActorFlags2_AlwaysPushJobs = 0x100,
    ActorFlags2_200_KeepAliveMaybe = 0x200,
    ActorFlags2_400 = 0x400,
    ActorFlags2_Armor = 0x800,
    ActorFlags2_NotXXX = 0x1000,
    ActorFlags2_ForbidSystemDeleteDistance = 0x2000,
    ActorFlags2_4000 = 0x4000,
    ActorFlags2_Delete = 0x8000,
    ActorFlags2_10000 = 0x10000,
    ActorFlags2_20000 = 0x20000,
    ActorFlags2_40000 = 0x40000,
    ActorFlags2_NoDistanceCheck2 = 0x80000,
    ActorFlags2_100000 = 0x100000,
    ActorFlags2_200000 = 0x200000,
    ActorFlags2_StasisableOrAllRadar = 0x400000,
    ActorFlags2_800000_NoUnloadForTurnActorBowCharge = 0x800000,
    ActorFlags2_1000000 = 0x1000000,
    ActorFlags2_2000000 = 0x2000000,
    ActorFlags2_4000000 = 0x4000000,
    ActorFlags2_8000000 = 0x8000000,
    ActorFlags2_10000000 = 0x10000000,
    ActorFlags2_20000000 = 0x20000000,
    ActorFlags2_40000000 = 0x40000000,
    ActorFlags2_80000000 = (int32_t)0x80000000,
};

enum ActorFlags3 : int32_t {
    ActorFlags3_DisableHideNonDemoMember = 0x1,
    ActorFlags3_2 = 0x2,
    ActorFlags3_4 = 0x4,
    ActorFlags3_8 = 0x8,
    ActorFlags3_10 = 0x10,
    ActorFlags3_20 = 0x20,
    ActorFlags3_40 = 0x40,
    ActorFlags3_80_KeepAliveMaybe = 0x80,
    ActorFlags3_100 = 0x100,
    ActorFlags3_200 = 0x200,
    ActorFlags3_400 = 0x400,
    ActorFlags3_Invisible = 0x800,
    ActorFlags3_1000 = 0x1000,
    ActorFlags3_DisableForbidJob = 0x2000,
    ActorFlags3_4000 = 0x4000,
    ActorFlags3_8000 = 0x8000,
    ActorFlags3_10000 = 0x10000,
    ActorFlags3_20000 = 0x20000,
    ActorFlags3_40000 = 0x40000,
    ActorFlags3_80000 = 0x80000,
    ActorFlags3_100000 = 0x100000,
    ActorFlags3_EmitCreateEffectMaybe = 0x200000,
    ActorFlags3_EmitDeleteEffectMaybe = 0x400000,
    ActorFlags3_800000 = 0x800000,
    ActorFlags3_ND = 0x1000000,
    ActorFlags3_GuardianOrRemainsOrBackseatKorok = 0x2000000,
    ActorFlags3_4000000 = 0x4000000,
    ActorFlags3_8000000 = 0x8000000,
    ActorFlags3_10000000 = 0x10000000,
    ActorFlags3_20000000 = 0x20000000,
    ActorFlags3_40000000 = 0x40000000,
    ActorFlags3_80000000 = (int32_t)0x80000000,
};

struct ActorWiiU : BaseProc {
    PADDED_BYTES(0xEC, 0xF0);
    BEType<uint32_t> physicsMainBodyPtr; // 0xF4
    BEType<uint32_t> physicsTgtBodyPtr;  // 0xF8
    uint8_t unk_FC[0x1F8 - 0xFC];
    BEMatrix34 mtx;
    uint32_t physicsMtxPtr;
    BEMatrix34 homeMtx;
    BEVec3 velocity;
    BEVec3 angularVelocity;
    BEVec3 scale; // 0x274
    BEType<float> dispDistSq;
    BEType<float> deleteDistSq;
    BEType<float> loadDistP10;
    BEVec3 previousPos;
    BEVec3 previousPos2;
    PADDED_BYTES(0x2A4, 0x324);
    BEType<uint32_t> modelBindInfoPtr;
    PADDED_BYTES(0x32C, 0x32C);
    BEType<uint32_t> gsysModelPtr;
    BEType<float> opacityOneRelated;
    BEType<float> startModelOpacity;
    BEType<float> modelOpacity;
    BEType<float> modelOpacityRelated;
    PADDED_BYTES(0x344, 0x348);
    struct {
        BEVec3 min;
        BEVec3 max;
    } aabb;
    BEType<ActorFlags2> flags2;
    BEType<ActorFlags2> flags2Copy;
    BEType<ActorFlags> flags;
    BEType<ActorFlags3> flags3; // 0x370 or 880. However in IDA there's a 0xF4 offset

    PADDED_BYTES(0x374, 0x390);
    BEType<uint32_t> asListPtr; // 0x394, ASListPartial
    PADDED_BYTES(0x398, 0x39C);
    BEType<uint32_t> actorPhysicsPtr; // 0x3A0
    PADDED_BYTES(0x3A4, 0x404);

    BEType<uint32_t> hashId;
    PADDED_BYTES(0x40C, 0x430);
    uint8_t unk_434;
    uint8_t unk_435;
    uint8_t opacityOrDoFlushOpacityToGPU;
    uint8_t unk_437;
    PADDED_BYTES(0x438, 0x440);
    BEType<uint32_t> actorX6A0Ptr;
    BEType<uint32_t> chemicalsPtr;
    BEType<uint32_t> reactionsPtr;
    PADDED_BYTES(0x450, 0x48C);
    BEType<float> lodDrawDistanceMultiplier;
    PADDED_BYTES(0x494, 0x538);
};
static_assert(offsetof(ActorWiiU, gsysModelPtr) == 0x330, "ActorWiiU.gsysModelPtr offset mismatch");
static_assert(offsetof(ActorWiiU, modelOpacity) == 0x33C, "ActorWiiU.modelOpacity offset mismatch");
static_assert(offsetof(ActorWiiU, modelOpacityRelated) == 0x340, "ActorWiiU.modelOpacityRelated offset mismatch");
static_assert(offsetof(ActorWiiU, opacityOrDoFlushOpacityToGPU) == 0x436, "ActorWiiU.opacityOrDoFlushOpacityToGPU offset mismatch");
static_assert(offsetof(ActorWiiU, velocity) == 0x25C, "ActorWiiU.velocity offset mismatch");
static_assert(offsetof(ActorWiiU, asListPtr) == 0x394, "ActorWiiU.asListPtr offset mismatch");
static_assert(offsetof(ActorWiiU, actorPhysicsPtr) == 0x3A0, "ActorWiiU.actorPhysicsPtr offset mismatch");
static_assert(sizeof(ActorWiiU) == 0x53C, "ActorWiiU size mismatch");

struct ASListDefinitionPartial {
    BEType<uint32_t> unk_00;
    sead::SafeString name;
};
static_assert(offsetof(ASListDefinitionPartial, name) == 0x04, "ASListDefinitionPartial.name offset mismatch");

// definitionPtr is only meaningful while assignedCount is set
struct ASListSlot : BETypeCompatible {
    BEType<uint32_t> definitionPtr;
    PADDED_BYTES(0x04, 0x08);
    BEType<uint32_t> assignedCount;
    PADDED_BYTES(0x10, 0x64);
};
static_assert(offsetof(ASListSlot, assignedCount) == 0x0C, "ASListSlot.assignedCount offset mismatch");
static_assert(sizeof(ASListSlot) == 0x68, "ASListSlot size mismatch");

struct ASListSlotGroup {
    PADDED_BYTES(0x00, 0x14);
    sead::Buffer slots;
    PADDED_BYTES(0x20, 0x30);
};
static_assert(offsetof(ASListSlotGroup, slots) == 0x18, "ASListSlotGroup.slots offset mismatch");
static_assert(sizeof(ASListSlotGroup) == 0x34, "ASListSlotGroup size mismatch");

struct ASListPartial {
    PADDED_BYTES(0x00, 0x90);
    sead::Buffer slotGroups;
};
static_assert(offsetof(ASListPartial, slotGroups) == 0x94, "ASListPartial.slotGroups offset mismatch");

struct DynamicActor : ActorWiiU {
    PADDED_BYTES(0x53C, 0x7C8);
};
static_assert(sizeof(DynamicActor) == 0x7CC, "DynamicActor size mismatch");

struct ActorWeapon {
    PADDED_BYTES(0x00, 0x0C);
};

struct ActorWeapons {
    static constexpr size_t CategoryCount = 6;

    ActorWeapon weapons[CategoryCount];
    BEType<uint32_t> actorThisPtr;
    BEType<uint32_t> actorWeaponsVtblPtr;
};
static_assert(sizeof(ActorWeapons) == 0x068, "ActorWeapons size mismatch");

struct PlayerOrEnemy : DynamicActor, ActorWeapons {
    BEType<float> float834;
};
static_assert(sizeof(PlayerOrEnemy) == 0x838, "PlayerOrEnemy size mismatch");

// 0x18000021 for carrying the electricity balls at least
// 0x00000010 for ladder
// 0x00000012 for ladder transition
// 0x18000002 for jumping
// 0x00008002 for gliding
// 0x00000090 for climbing
// 0x00000410 for swimming (00001000001000000000000000000000)

// Observed to be related to animations overrides.
// Some player actions are represented by combinaisons of these flags.
// No flags are set when in an event (interaction with npc, merchants, talks etc)
enum class PlayerMoveBitFlags : uint32_t {
    IS_MOVING = 1 << 0,
    IS_IN_AIR = 1 << 1,
    IS_CROUCHING = 1 << 2,
    UNK_008 = 1 << 3,
    IS_SWIMMING_OR_CLIMBING = 1 << 4, // This used alone = climbing ladder. 
    IS_PICKING_DROPPING_THROWING_OBJECT = 1 << 5,
    UNK_064 = 1 << 6,
    IS_CLIMBING_WALL = 1 << 7, // This + IS_SWIMMING_OR_CLIMBING = IS_CLIMBING_WALL 
    UNK_256 = 1 << 8,
    UNK_512 = 1 << 9,
    IS_SWIMMING = 1 << 10, // This + IS_SWIMMING_OR_CLIMBING = IS_SWIMMING
    IS_SWIMMING_DASH = 1 << 11, // This + IS_SWIMMING_OR_CLIMBING + IS_SWIMMING = IS_SWIMMING_DASH
    UNK_4096 = 1 << 12,
    UNK_8192 = 1 << 13,
    UNK_16384 = 1 << 14,
    IS_GLIDER_ACTIVE = 1 << 15,
    UNK_65536 = 1 << 16,
    IS_RIDING_SAND_SEAL = 1 << 17,
    UNK_262144 = 1 << 18,
    IS_RAGDOLL_ACTIVE = 1 << 19,
    IS_RIDING_HORSE = 1 << 20,
    UNK_2097152 = 1 << 21,
    UNK_4194304 = 1 << 22,
    UNK_8388608 = 1 << 23,
    UNK_16777216 = 1 << 24,
    UNK_33554432 = 1 << 25,
    UNK_67108864 = 1 << 26,
    EVENT_UNK_134217728 = 1 << 27, // GETS DISABLED WHEN INSIDE AN EVENT
    EVENT2_UNK_268435456 = 1 << 28,
    UNK_536870912 = 1 << 29,
    UNK_1073741824 = 1 << 30,
    GET_HIT_MAYBE_UNABLE_TO_INTERACT = 1u << 31
};

ENABLE_BITMASK_OPERATORS(PlayerMoveBitFlags);

struct PlayerBase : PlayerOrEnemy {
    PADDED_BYTES(0x838, 0x8D8);
    BEType<PlayerMoveBitFlags> moveBitFlags;
    PADDED_BYTES(0x8E0, 0x12A4);
};
static_assert(offsetof(PlayerBase, moveBitFlags) == 0x8DC, "Player.float834 offset mismatch");
static_assert(sizeof(PlayerBase) == 0x12A8, "PlayerBase size mismatch");

struct BaseProcLink {
    BEType<uint32_t> baseProcLinkData;
    BEType<uint32_t> id;
    BEType<uint8_t> acquired;
    uint8_t padding9[3];
};
static_assert(offsetof(BaseProcLink, id) == 0x4, "BaseProcLink.id offset mismatch");
static_assert(offsetof(BaseProcLink, acquired) == 0x8, "BaseProcLink.acquired offset mismatch");
static_assert(sizeof(BaseProcLink) == 0xC, "BaseProcLink size mismatch");

struct BaseProcLinkData {
    uint8_t criticalSection[0x3C];
    BEType<uint32_t> id;
    BEType<uint32_t> actor;
    BEType<uint32_t> refCount;
};
static_assert(offsetof(BaseProcLinkData, id) == 0x3C, "BaseProcLinkData.id offset mismatch");
static_assert(offsetof(BaseProcLinkData, actor) == 0x40, "BaseProcLinkData.actor offset mismatch");
static_assert(sizeof(BaseProcLinkData) == 0x48, "BaseProcLinkData size mismatch");

struct BaseProcHandle {
    BEType<uint32_t> unit;
    BEType<uint8_t> failed;
    uint8_t padding5[3];
};
static_assert(sizeof(BaseProcHandle) == 0x8, "BaseProcHandle size mismatch");

struct PlayerArmors {
    BEType<uint32_t> vtable;
    BEType<uint32_t> unk_04;
    BaseProcLink armorHead;
    BaseProcLink armorBody;
    BaseProcLink armorLower;
    BaseProcLink armorExtra[3];
    BaseProcHandle creationHandles[6];
    PADDED_BYTES(0x80, 0xAC);
    BEType<uint32_t> owner;
    PADDED_BYTES(0xB4, 0xE4);
};
static_assert(offsetof(PlayerArmors, armorHead) == 0x08, "PlayerArmors.armorHead offset mismatch");
static_assert(offsetof(PlayerArmors, owner) == 0xB0, "PlayerArmors.owner offset mismatch");
static_assert(sizeof(PlayerArmors) == 0xE8, "PlayerArmors size mismatch");

struct PlayerActor : PlayerBase {
    PADDED_BYTES(0x12A8, 0x1D90);
    PlayerArmors armors;
    PADDED_BYTES(0x1E7C, 0x2524);
};
static_assert(offsetof(PlayerActor, armors) == 0x1D94, "PlayerActor.armors offset mismatch");
static_assert(sizeof(PlayerActor) == 0x2528, "PlayerActor size mismatch");

struct WeaponBase : ActorWiiU {
    PADDED_BYTES(0x53C, 0x574);
    BaseProcLink requestedParentActorLink;
    PADDED_BYTES(0x584, 0x5F0);
    BEType<int32_t> requestedActorWeaponsCategory;
    BEType<uint8_t> isEquippedProbably;
    BEType<uint8_t> field_5FD;
    BEType<uint8_t> field_5FE;
    BEType<uint8_t> field_5FF;
    PADDED_BYTES(0x5FC, 0x60C);
    BaseProcLink activeParentActorLink;
    BaseProcLink previousParentActorLink;
    BaseProcLink field_628;
    PADDED_BYTES(0x634, 0x6A0);
    BEType<int32_t> activeActorWeaponsCategory;
    PADDED_BYTES(0x6A8, 0x728);
};
static_assert(offsetof(WeaponBase, requestedParentActorLink) == 0x578, "WeaponBase.requestedParentActorLink offset mismatch");
static_assert(offsetof(WeaponBase, requestedActorWeaponsCategory) == 0x5F4, "WeaponBase.requestedActorWeaponsCategory offset mismatch");
static_assert(offsetof(WeaponBase, isEquippedProbably) == 0x5F8, "WeaponBase.isEquippedProbably offset mismatch");
static_assert(offsetof(WeaponBase, activeParentActorLink) == 0x610, "WeaponBase.activeParentActorLink offset mismatch");
static_assert(offsetof(WeaponBase, previousParentActorLink) == 0x61C, "WeaponBase.previousParentActorLink offset mismatch");
static_assert(offsetof(WeaponBase, field_628) == 0x628, "WeaponBase.field_628 offset mismatch");
static_assert(offsetof(WeaponBase, activeActorWeaponsCategory) == 0x6A4, "WeaponBase.activeActorWeaponsCategory offset mismatch");
static_assert(sizeof(WeaponBase) == 0x72C, "WeaponBase size mismatch");

struct Struct20 {
    BEType<uint32_t> __vftable;
    BEType<uint32_t> field_4;
    BEType<uint32_t> field_8;
    BEType<uint32_t> field_C;
    BEType<uint32_t> field_10;
    BEType<uint32_t> field_14;
};

struct DamageMgr {
    BEType<uint32_t> struct20Ptr;
    BEType<uint32_t> struct20PlayerRelatedPtr;
    BEType<uint32_t> actor;
    struct {
        BEType<uint32_t> size;
        BEType<uint32_t> pointer;
    } damageCallbacks;
    BEType<uint32_t> field_14;
    BEType<uint8_t> field_18;
    BEType<uint8_t> field_19;
    BEType<uint8_t> field_1A;
    BEType<uint8_t> field_1B;
    BEType<uint32_t> __vftable;
    BEType<uint32_t> deleter;
    BEType<uint32_t> field_24;
    BEType<uint32_t> damage;
    BEType<uint32_t> field_2C;
    BEType<uint32_t> minDmg;
    BEType<uint32_t> field_34;
    BEType<uint32_t> field_38;
    BEType<uint32_t> field_3C;
    BEType<uint32_t> damageType;
    BEType<uint32_t> field_44;
    BEType<uint8_t> field_48;
    BEType<uint8_t> field_49;
    BEType<uint8_t> field_4A;
    BEType<uint8_t> field_4B;
};
static_assert(sizeof(DamageMgr) == 0x4C, "DamageMgr size mismatch");

struct AttackSensorInitArg {
    BEType<uint32_t> mode;
    BEType<uint32_t> flags;
    BEType<float> multiplier;
    BEType<float> scale;
    BEType<uint32_t> shieldBreakPower;
    BEType<uint8_t> overrideImpact;
    BEType<uint8_t> unk_15;
    BEType<uint8_t> unk_16;
    BEType<uint8_t> unk_17;
    BEType<uint32_t> powerForPlayers;
    BEType<uint32_t> impact;
    BEType<uint32_t> unk_20;
    BEType<uint32_t> comboCount;
    BEType<uint8_t> isContactLayerInitialized;
    BEType<uint8_t> field_87C;
    BEType<uint8_t> field_87D;
    BEType<uint8_t> field_87E;
    BEType<uint8_t> resetAttack;
    BEType<uint8_t> field_8A2;
    BEType<uint8_t> field_8A3;
    BEType<uint8_t> field_8A4;
};
static_assert(sizeof(AttackSensorInitArg) == 0x30, "AttackSensorInitArg size mismatch");

struct AttackSensorOtherArg {
    BEType<uint32_t> flags;
    BEType<uint32_t> shieldBreakPower;
    BEType<uint8_t> overrideImpact;
    BEType<uint8_t> field_09;
    BEType<uint8_t> field_0A;
    BEType<uint8_t> field_0B;
    BEType<float> scale;
    BEType<float> multiplier;
    BEType<uint8_t> resetAttack;
    BEType<uint8_t> wasAttackMode;
    BEType<uint8_t> field_12;
    BEType<uint8_t> field_13;
    BEType<uint32_t> powerForPlayers;
    BEType<uint32_t> impact;
    BEType<uint32_t> comboCount;
};
static_assert(sizeof(AttackSensorOtherArg) == 0x24, "AttackSensorOtherArg size mismatch");

enum class WeaponType : uint32_t {
    SmallSword = 0x0,
    LargeSword = 0x1,
    Spear = 0x2,
    Bow = 0x3,
    Shield = 0x4,
    UnknownWeapon = 0x5,
};

enum class EquipType {
    None = 0,
    Melee = 1,
    Shield = 2,
    Bow = 3,
    Arrow = 4,
    SheikahSlate = 5,
    MagnetGlove = 6,
    ThrowableObject = 7
};

enum class RumbleType {
    Fixed,
    Raising,
    Falling,
    OscillationSmooth,
    OscillationFallingSawtoothWave,
    OscillationRaisingSawtoothWave
};

 struct RumbleParameters {
     bool prioritizeThisRumble = false;
     int hand = 0;
     RumbleType rumbleType = RumbleType::Fixed;
     float oscillationFrequency = 0.0f;
     bool keepRumblingOnEffectEnd = false; // requires stopInputsRumble() to manually stop the rumble
     double effectDuration = 0;
     float frequency = 0.0f;
     float amplitude = 0.0f;
 };

struct Weapon : WeaponBase {
    PADDED_BYTES(0x72C, 0x870);
    AttackSensorInitArg setupAttackSensor;
    PADDED_BYTES(0x8A4, 0x934);
    BEType<WeaponType> type;
    AttackSensorOtherArg finalizedAttackSensor;
    BEType<uint32_t> weaponShockwaves;
    PADDED_BYTES(0x964, 0x96C);
    BEType<uint32_t> field_974_triggerEventsMaybe;
    BEVec3 field_978;
    BEType<uint32_t> field_984;
    BEType<uint32_t> field_988;
    BEType<uint32_t> field_98C;
    BEType<uint32_t> field_990;
    BEVec3 originalScale;
    BEType<uint32_t> field_99C;
    BEType<uint32_t> field_9A0;
    BEType<uint32_t> field_9A4;
    BEType<uint32_t> field_9A8;
    BEType<uint32_t> field_9AC;
    BEType<uint32_t> field_9B0;
    BEType<uint32_t> field_9B4;
    struct {
        PADDED_BYTES(0x00, 0x08);
        BEType<uint32_t> struct7Ptr; // stores attackinfo, so prolly post-hit stuff
        BEType<uint32_t> field_10;
        BEType<uint32_t> field_14;
        BEType<uint32_t> field_18;
        BEType<uint32_t> field_1C;
        BEType<uint32_t> attackSensorPtr;
        BEType<uint32_t> struct8Ptr; // stores attackinfo, so prolly post-hit stuff. seems to be more player specific
        BEType<uint32_t> field_28;
        BEType<uint32_t> field_2C;
        BEType<uint32_t> field_30;
        BEType<uint32_t> field_34;
        BEType<uint32_t> attackSensorStruct8Ptr;
        BEType<uint8_t> field_3C;
        PADDED_BYTES(0x3D, 0x3C);
    } actorAtk;
    BEType<float> field_9F8;
    BEType<float> field_9FC;
    BEType<uint32_t> damageMgrPtr;
    PADDED_BYTES(0xA04, 0xA10);
    BEType<uint16_t> weaponFlags;
    BEType<uint16_t> otherFlags;
    PADDED_BYTES(0xA18, 0xB58);
};
static_assert(offsetof(Weapon, setupAttackSensor.resetAttack) == 0x8A0, "Weapon.setupAttackSensor.resetAttack offset mismatch");
static_assert(offsetof(Weapon, setupAttackSensor.mode) == 0x874, "Weapon.setupAttackSensor.mode offset mismatch");
static_assert(offsetof(Weapon, finalizedAttackSensor.resetAttack) == 0x950, "Weapon.finalizedAttackSensor.resetAttack offset mismatch");
static_assert(offsetof(Weapon, field_99C) == 0x99C, "Weapon.field_99C offset mismatch");
static_assert(offsetof(Weapon, field_9A0) == 0x9A0, "Weapon.field_9A0 offset mismatch");
static_assert(offsetof(Weapon, weaponFlags) == 0xA14, "Weapon.weaponFlags offset mismatch");
static_assert(sizeof(Weapon) == 0xB5C, "Weapon size mismatch");

struct LookAtMatrix {
    BEVec3 pos;
    BEVec3 target;
    BEVec3 up;
    BEVec3 unknown;
    BEType<float> zNear;
    BEType<float> zFar;
};

struct ActCamera : ActorWiiU {
    BEType<uint32_t> dword53C;
    BEType<float> float540;
    PADDED_BYTES(0x544, 0x54C);
    LookAtMatrix origCamMtx;
    PADDED_BYTES(0x588, 0x5BC);
    LookAtMatrix finalCamMtx;
};
static_assert(offsetof(ActCamera, origCamMtx) == 0x550, "ActCamera.origCamMtx offset mismatch");
static_assert(offsetof(ActCamera, finalCamMtx) == 0x5C0, "ActCamera.finalCamMtx offset mismatch");

struct BESeadCamera {
    BEMatrix34 mtx;
    BEType<uint32_t> __vftable;
};
struct BESeadLookAtCamera : BESeadCamera {
    BEVec3 pos;
    BEVec3 at;
    BEVec3 up;

    bool operator==(const BESeadLookAtCamera& other) const {
        return pos == other.pos && at == other.at && up == other.up;
    }
};
static_assert(sizeof(BESeadCamera) == 0x34, "BESeadCamera size mismatch");
static_assert(sizeof(BESeadLookAtCamera) == 0x58, "BESeadLookAtCamera size mismatch");


struct UIManagerInnerArray {
    PADDED_BYTES(0x00, 0x34);
    BEVec3 uiPos1;
    BEVec3 uiPos2;
    PADDED_BYTES(0x50, 0x40668-0x04);
};
struct UIManager {
    PADDED_BYTES(0x00, 0x1C);
    UIManagerInnerArray innerArray;
    PADDED_BYTES(0x40688, 0x40DD0-0x04);
};

static_assert(offsetof(UIManager, innerArray) == 0x20, "UIManager.innerArray offset mismatch");
static_assert(offsetof(UIManager, innerArray.uiPos1) == 0x58, "UIManagerInnerArray.uiPos1 offset mismatch");
static_assert(offsetof(UIManager, innerArray.uiPos2) == 0x64, "UIManagerInnerArray.uiPos1 offset mismatch");
static_assert(sizeof(UIManagerInnerArray) == 0x40668, "UIManagerInnerArray size mismatch");
static_assert(sizeof(UIManager) == 0x40DD0, "UIManager size mismatch");

// not identical memory layout wise
struct Frustum {
    glm::vec4 planes[6];

    void update(const glm::mat4& vp) {
        // Left
        planes[0] = glm::vec4(vp[0][3] + vp[0][0], vp[1][3] + vp[1][0], vp[2][3] + vp[2][0], vp[3][3] + vp[3][0]);
        // Right
        planes[1] = glm::vec4(vp[0][3] - vp[0][0], vp[1][3] - vp[1][0], vp[2][3] - vp[2][0], vp[3][3] - vp[3][0]);
        // Bottom
        planes[2] = glm::vec4(vp[0][3] + vp[0][1], vp[1][3] + vp[1][1], vp[2][3] + vp[2][1], vp[3][3] + vp[3][1]);
        // Top
        planes[3] = glm::vec4(vp[0][3] - vp[0][1], vp[1][3] - vp[1][1], vp[2][3] - vp[2][1], vp[3][3] - vp[3][1]);
        // Near
        planes[4] = glm::vec4(vp[0][3] + vp[0][2], vp[1][3] + vp[1][2], vp[2][3] + vp[2][2], vp[3][3] + vp[3][2]);
        // Far
        planes[5] = glm::vec4(vp[0][3] - vp[0][2], vp[1][3] - vp[1][2], vp[2][3] - vp[2][2], vp[3][3] - vp[3][2]);

        for (int i = 0; i < 6; ++i) {
            float len = glm::length(glm::vec3(planes[i]));
            planes[i] /= len;
        }
    }

    bool checkSphere(const glm::vec3& center, float radius) const {
        for (int i = 0; i < 6; ++i) {
            float dist = glm::dot(glm::vec3(planes[i]), center) + planes[i].w;
            if (dist < -radius) {
                return false;
            }
        }
        return true;
    }
};

namespace ksys::phys {
    enum GroundHit {
        Player = 0x0,
        Animal = 0x1,
        NPC = 0x2,
        Camera = 0x3,
        AttackHitPlayer = 0x4,
        AttackHitEnemy = 0x5,
        Arrow = 0x6,
        Bomb = 0x7,
        Magnet = 0x8,
        CameraBody = 0x9,
        IK = 0xA,
        Grudge = 0xB,
        MovingTrolley = 0xC,
        LineOfSight = 0xD,
        Giant = 0xE,
        HitAll = 0xF,
        Ignore = 0x10,
    };

    struct RayCast {
        BEVec3 from;
        BEVec3 to;
        BEType<uint32_t> groupHandlerPtr;
        GroundHit groundHit;
        uint8_t gap20[40];
        char char48;
        uint8_t gap49[7];
        BEType<uint32_t> dword50;
        BEType<uint32_t> dword54;
        sead::PtrArrayImpl mLayerMasks;
        BEType<uint32_t> dword64;
        uint8_t byte68;
        uint8_t byte69;
        uint8_t byte6A;
        sead::PtrArrayImpl mIgnoredGroups;
        uint8_t gap78[16];
        BEType<uint32_t> dword88;
        BEType<uint32_t> dword8C;
        BEType<uint32_t> __vftable;
    };

    struct RayCastBodyQuery : RayCast {
        BEType<uint32_t> mHitRigidBody;
    };
}

struct InlineParamVec3 {
    BEType<uint32_t> intOrFloatOrBool;
    BaseProcLink procLink;
    BEVec3 value;
    BEType<uint8_t> gap1C[16];
    BEType<uint32_t> type;
    BEType<uint32_t> keyPtr;
};

struct InlineParamBool {
    BEType<uint8_t> value;
    BEType<uint8_t> gap1[3];
    BaseProcLink procLink;
    BEType<uint8_t> gap1C[16+4+4+4];
    BEType<uint32_t> type;
    BEType<uint32_t> keyPtr;
};

struct InlineParamFloat {
    BEType<float> value;
    BaseProcLink procLink;
    BEType<uint8_t> gap1C[16+4+4+4];
    BEType<uint32_t> type;
    BEType<uint32_t> keyPtr;
};

static_assert(sizeof(InlineParamVec3) == 0x34, "InlineParamVec3 size mismatch");
static_assert(sizeof(InlineParamBool) == 0x34, "InlineParamBool size mismatch");
static_assert(sizeof(InlineParamFloat) == 0x34, "InlineParamFloat size mismatch");



#pragma pack(pop)

inline std::string contactLayerNames[] = {
    "EntityObject",
    "EntitySmallObject",
    "EntityGroundObject",
    "EntityPlayer",
    "EntityNPC",
    "EntityRagdoll",
    "EntityWater",
    "EntityAirWall",
    "EntityGround",
    "EntityGroundSmooth",
    "EntityGroundRough",
    "EntityRope",
    "EntityTree",
    "EntityNPC_NoHitPlayer",
    "EntityHitOnlyWater",
    "EntityWallForClimb",
    "EntityHitOnlyGround",
    "EntityQueryCustomReceiver",
    "EntityForbidden18",
    "EntityNoHit",
    "EntityMeshVisualizer",
    "EntityForbidden21",
    "EntityForbidden22",
    "EntityForbidden23",
    "EntityForbidden24",
    "EntityForbidden25",
    "EntityForbidden26",
    "EntityForbidden27",
    "EntityForbidden28",
    "EntityForbidden29",
    "EntityForbidden30",
    "EntityEnd",

    "SensorObject",
    "SensorSmallObject",
    "SensorPlayer",
    "SensorEnemy",
    "SensorNPC",
    "SensorHorse",
    "SensorRope",
    "SensorAttackPlayer",
    "SensorAttackEnemy",
    "SensorChemical",
    "SensorTerror",
    "SensorHitOnlyInDoor",
    "SensorInDoor",
    "SensorReserve13",
    "SensorReserve14",
    "SensorChemicalElement",
    "SensorAttackCommon",
    "SensorQueryOnly",
    "SensorTree",
    "SensorCamera",
    "SensorMeshVisualizer",
    "SensorNoHit",
    "SensorReserve20",
    "SensorCustomReceiver",
    "SensorEnd"
};

enum class ContactLayer : uint32_t {
    EntityObject = 0,
    EntitySmallObject,
    EntityGroundObject,
    EntityPlayer,
    EntityNPC,
    EntityRagdoll,
    EntityWater,
    EntityAirWall,
    EntityGround,
    EntityGroundSmooth,
    EntityGroundRough,
    EntityRope,
    EntityTree,
    EntityNPC_NoHitPlayer,
    EntityHitOnlyWater,
    EntityWallForClimb,
    EntityHitOnlyGround,
    EntityQueryCustomReceiver,
    EntityForbidden18,
    EntityNoHit,
    EntityMeshVisualizer,
    EntityForbidden21,
    EntityForbidden22,
    EntityForbidden23,
    EntityForbidden24,
    EntityForbidden25,
    EntityForbidden26,
    EntityForbidden27,
    EntityForbidden28,
    EntityForbidden29,
    EntityForbidden30,
    EntityEnd,

    SensorObject,
    SensorSmallObject,
    SensorPlayer,
    SensorEnemy,
    SensorNPC,
    SensorHorse,
    SensorRope,
    SensorAttackPlayer,
    SensorAttackEnemy,
    SensorChemical,
    SensorTerror,
    SensorHitOnlyInDoor,
    SensorInDoor,
    SensorReserve13,
    SensorReserve14,
    SensorChemicalElement,
    SensorAttackCommon,
    SensorQueryOnly,
    SensorTree,
    SensorCamera,
    SensorMeshVisualizer,
    SensorNoHit,
    SensorReserve20,
    SensorCustomReceiver,
    SensorEnd
};


enum class ScreenId {
    ScreenId_START = 0x0,

    GamePadBG_00 = 0x0,
    Title_00 = 0x1,
    MainScreen3D_00 = 0x2,
    Message3D_00 = 0x3,
    AppCamera_00 = 0x4,
    WolfLinkHeartGauge_00 = 0x5,
    EnergyMeterDLC_00 = 0x6,
    MainHorse_00 = 0x7,
    MiniGame_00 = 0x8,
    ReadyGo_00 = 0x9,
    KeyNum_00 = 0xA,
    MessageGet_00 = 0xB,
    DoCommand_00 = 0xC,
    SousaGuide_00 = 0xD,
    GameTitle_00 = 0xE,
    DemoName_00 = 0xF,
    DemoNameEnemy_00 = 0x10,
    MessageSp_00_NoTop = 0x11,
    ShopBG_00 = 0x12,
    ShopBtnList5_00 = 0x13,
    ShopBtnList20_00 = 0x14,
    ShopBtnList15_00 = 0x15,
    ShopInfo_00 = 0x16,
    ShopHorse_00 = 0x17,
    Rupee_00 = 0x18,
    KologNum_00 = 0x19,
    AkashNum_00 = 0x1A,
    MamoNum_00 = 0x1B,
    Time_00 = 0x1C,
    PauseMenuBG_00 = 0x1D,
    SeekPadMenuBG_00 = 0x1E,
    AppTool_00 = 0x1F,
    AppAlbum_00 = 0x20,
    AppPictureBook_00 = 0x21,
    AppMapDungeon_00 = 0x22,
    MainScreenMS_00 = 0x23,
    MainScreenHeartIchigekiDLC_00 = 0x24,
    MainScreen_00 = 0x25,
    MainDungeon_00 = 0x26,
    ChallengeWin_00 = 0x27,
    PickUp_00 = 0x28,
    MessageTipsRunTime_00 = 0x29,
    AppMap_00 = 0x2A,
    AppSystemWindowNoBtn_00 = 0x2B,
    AppHome_00 = 0x2C,
    MainShortCut_00 = 0x2D,
    PauseMenu_00 = 0x2E,
    PauseMenuInfo_00 = 0x2F,
    GameOver_00 = 0x30,
    HardMode_00 = 0x31,
    SaveTransferWindow_00 = 0x32,
    MessageTipsPauseMenu_00 = 0x33,
    MessageTips_00 = 0x34,
    OptionWindow_00 = 0x35,
    AmiiboWindow_00 = 0x36,
    SystemWindowNoBtn_00 = 0x37,
    ControllerWindow_00 = 0x38,
    SystemWindow_01 = 0x39,
    SystemWindow_00 = 0x3A,
    PauseMenuRecipe_00 = 0x3B,
    PauseMenuMantan_00 = 0x3C,
    PauseMenuEiketsu_00 = 0x3D,
    AppSystemWindow_00 = 0x3E,
    DLCWindow_00 = 0x3F,
    HardModeTextDLC_00 = 0x40,
    TestButton = 0x41,
    TestPocketUIDRC = 0x42,
    TestPocketUITV = 0x43,
    BoxCursorTV = 0x44,
    FadeDemo_00 = 0x45,
    StaffRoll_00 = 0x46,
    StaffRollDLC_00 = 0x47,
    End_00 = 0x48,
    DLCSinJuAkashiNum_00 = 0x49,
    MessageDialog = 0x4A,
    DemoMessage = 0x4B,
    MessageSp_00 = 0x4C,
    Thanks_00 = 0x4D,
    Fade = 0x4E,
    KeyBoradTextArea_00 = 0x4F,
    LastComplete_00 = 0x50,
    OPtext_00 = 0x51,
    LoadingWeapon_00 = 0x52,
    MainHardMode_00 = 0x53,
    LoadSaveIcon_00 = 0x54,
    FadeStatus_00 = 0x55,
    Skip_00 = 0x56,
    ChangeController_00 = 0x57,
    ChangeControllerDRC_00 = 0x58,
    DemoStart_00 = 0x59,
    BootUp_00 = 0x5A,
    BootUp_00_2 = 0x5B,
    ChangeControllerNN_00 = 0x5C,
    AppMenuBtn_00 = 0x5D,
    HomeMenuCapture_00 = 0x5E,
    HomeMenuCaptureDRC_00 = 0x5F,
    HomeNixSign_00 = 0x60,
    ErrorViewer_00 = 0x61,
    ErrorViewerDRC_00 = 0x62,

    ScreenId_END = ErrorViewerDRC_00
};

static const char* ScreenIdToString(ScreenId e) {
    switch (e) {
        case ScreenId::GamePadBG_00:
            return "GamePadBG_00";
        case ScreenId::Title_00:
            return "Title_00";
        case ScreenId::MainScreen3D_00:
            return "MainScreen3D_00";
        case ScreenId::Message3D_00:
            return "Message3D_00";
        case ScreenId::AppCamera_00:
            return "AppCamera_00";
        case ScreenId::WolfLinkHeartGauge_00:
            return "WolfLinkHeartGauge_00";
        case ScreenId::EnergyMeterDLC_00:
            return "EnergyMeterDLC_00";
        case ScreenId::MainHorse_00:
            return "MainHorse_00";
        case ScreenId::MiniGame_00:
            return "MiniGame_00";
        case ScreenId::ReadyGo_00:
            return "ReadyGo_00";
        case ScreenId::KeyNum_00:
            return "KeyNum_00";
        case ScreenId::MessageGet_00:
            return "MessageGet_00";
        case ScreenId::DoCommand_00:
            return "DoCommand_00";
        case ScreenId::SousaGuide_00:
            return "SousaGuide_00";
        case ScreenId::GameTitle_00:
            return "GameTitle_00";
        case ScreenId::DemoName_00:
            return "DemoName_00";
        case ScreenId::DemoNameEnemy_00:
            return "DemoNameEnemy_00";
        case ScreenId::MessageSp_00_NoTop:
            return "MessageSp_00_NoTop";
        case ScreenId::ShopBG_00:
            return "ShopBG_00";
        case ScreenId::ShopBtnList5_00:
            return "ShopBtnList5_00";
        case ScreenId::ShopBtnList20_00:
            return "ShopBtnList20_00";
        case ScreenId::ShopBtnList15_00:
            return "ShopBtnList15_00";
        case ScreenId::ShopInfo_00:
            return "ShopInfo_00";
        case ScreenId::ShopHorse_00:
            return "ShopHorse_00";
        case ScreenId::Rupee_00:
            return "Rupee_00";
        case ScreenId::KologNum_00:
            return "KologNum_00";
        case ScreenId::AkashNum_00:
            return "AkashNum_00";
        case ScreenId::MamoNum_00:
            return "MamoNum_00";
        case ScreenId::Time_00:
            return "Time_00";
        case ScreenId::PauseMenuBG_00:
            return "PauseMenuBG_00";
        case ScreenId::SeekPadMenuBG_00:
            return "SeekPadMenuBG_00";
        case ScreenId::AppTool_00:
            return "AppTool_00";
        case ScreenId::AppAlbum_00:
            return "AppAlbum_00";
        case ScreenId::AppPictureBook_00:
            return "AppPictureBook_00";
        case ScreenId::AppMapDungeon_00:
            return "AppMapDungeon_00";
        case ScreenId::MainScreenMS_00:
            return "MainScreenMS_00";
        case ScreenId::MainScreenHeartIchigekiDLC_00:
            return "MainScreenHeartIchigekiDLC_00";
        case ScreenId::MainScreen_00:
            return "MainScreen_00";
        case ScreenId::MainDungeon_00:
            return "MainDungeon_00";
        case ScreenId::ChallengeWin_00:
            return "ChallengeWin_00";
        case ScreenId::PickUp_00:
            return "PickUp_00";
        case ScreenId::MessageTipsRunTime_00:
            return "MessageTipsRunTime_00";
        case ScreenId::AppMap_00:
            return "AppMap_00";
        case ScreenId::AppSystemWindowNoBtn_00:
            return "AppSystemWindowNoBtn_00";
        case ScreenId::AppHome_00:
            return "AppHome_00";
        case ScreenId::MainShortCut_00:
            return "MainShortCut_00";
        case ScreenId::PauseMenu_00:
            return "PauseMenu_00";
        case ScreenId::PauseMenuInfo_00:
            return "PauseMenuInfo_00";
        case ScreenId::GameOver_00:
            return "GameOver_00";
        case ScreenId::HardMode_00:
            return "HardMode_00";
        case ScreenId::SaveTransferWindow_00:
            return "SaveTransferWindow_00";
        case ScreenId::MessageTipsPauseMenu_00:
            return "MessageTipsPauseMenu_00";
        case ScreenId::MessageTips_00:
            return "MessageTips_00";
        case ScreenId::OptionWindow_00:
            return "OptionWindow_00";
        case ScreenId::AmiiboWindow_00:
            return "AmiiboWindow_00";
        case ScreenId::SystemWindowNoBtn_00:
            return "SystemWindowNoBtn_00";
        case ScreenId::ControllerWindow_00:
            return "ControllerWindow_00";
        case ScreenId::SystemWindow_01:
            return "SystemWindow_01";
        case ScreenId::SystemWindow_00:
            return "SystemWindow_00";
        case ScreenId::PauseMenuRecipe_00:
            return "PauseMenuRecipe_00";
        case ScreenId::PauseMenuMantan_00:
            return "PauseMenuMantan_00";
        case ScreenId::PauseMenuEiketsu_00:
            return "PauseMenuEiketsu_00";
        case ScreenId::AppSystemWindow_00:
            return "AppSystemWindow_00";
        case ScreenId::DLCWindow_00:
            return "DLCWindow_00";
        case ScreenId::HardModeTextDLC_00:
            return "HardModeTextDLC_00";
        case ScreenId::TestButton:
            return "TestButton";
        case ScreenId::TestPocketUIDRC:
            return "TestPocketUIDRC";
        case ScreenId::TestPocketUITV:
            return "TestPocketUITV";
        case ScreenId::BoxCursorTV:
            return "BoxCursorTV";
        case ScreenId::FadeDemo_00:
            return "FadeDemo_00";
        case ScreenId::StaffRoll_00:
            return "StaffRoll_00";
        case ScreenId::StaffRollDLC_00:
            return "StaffRollDLC_00";
        case ScreenId::End_00:
            return "End_00";
        case ScreenId::DLCSinJuAkashiNum_00:
            return "DLCSinJuAkashiNum_00";
        case ScreenId::MessageDialog:
            return "MessageDialog";
        case ScreenId::DemoMessage:
            return "DemoMessage";
        case ScreenId::MessageSp_00:
            return "MessageSp_00";
        case ScreenId::Thanks_00:
            return "Thanks_00";
        case ScreenId::Fade:
            return "Fade";
        case ScreenId::KeyBoradTextArea_00:
            return "KeyBoradTextArea_00";
        case ScreenId::LastComplete_00:
            return "LastComplete_00";
        case ScreenId::OPtext_00:
            return "OPtext_00";
        case ScreenId::LoadingWeapon_00:
            return "LoadingWeapon_00";
        case ScreenId::MainHardMode_00:
            return "MainHardMode_00";
        case ScreenId::LoadSaveIcon_00:
            return "LoadSaveIcon_00";
        case ScreenId::FadeStatus_00:
            return "FadeStatus_00";
        case ScreenId::Skip_00:
            return "Skip_00";
        case ScreenId::ChangeController_00:
            return "ChangeController_00";
        case ScreenId::ChangeControllerDRC_00:
            return "ChangeControllerDRC_00";
        case ScreenId::DemoStart_00:
            return "DemoStart_00";
        case ScreenId::BootUp_00:
            return "BootUp_00";
        case ScreenId::BootUp_00_2:
            return "BootUp_00_2";
        case ScreenId::ChangeControllerNN_00:
            return "ChangeControllerNN_00";
        case ScreenId::AppMenuBtn_00:
            return "AppMenuBtn_00";
        case ScreenId::HomeMenuCapture_00:
            return "HomeMenuCapture_00";
        case ScreenId::HomeMenuCaptureDRC_00:
            return "HomeMenuCaptureDRC_00";
        case ScreenId::HomeNixSign_00:
            return "HomeNixSign_00";
        case ScreenId::ErrorViewer_00:
            return "ErrorViewer_00";
        case ScreenId::ErrorViewerDRC_00:
            return "ErrorViewerDRC_00";
        default:
            return "unknown";
    }
}
