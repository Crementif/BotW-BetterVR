#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "cemu/guest_memory.h"
#include "game/botw/player/player.h"

class GameUtils {
public:

    struct AnimationSlot {
        uint32_t groupIndex;
        uint32_t slotIndex;
        uint32_t slotPtr;
        std::string name;
    };

    static std::vector<AnimationSlot> GetAnimationSlots(uint32_t asListPtr) {
        std::vector<AnimationSlot> animations;
        if (asListPtr == 0) {
            return animations;
        }

        sead::Buffer groups = Guest::Get<sead::Buffer>(asListPtr + offsetof(ASListPartial, slotGroups));
        const uint32_t groupsPtr = groups.data.getLE();
        const int32_t groupCount = groups.size.getLE();
        if (groupsPtr == 0) {
            return animations;
        }

        for (int32_t groupIdx = 0; groupIdx < groupCount; groupIdx++) {
            uint32_t groupPtr = groupsPtr + static_cast<uint32_t>(groupIdx) * sizeof(ASListSlotGroup);
            sead::Buffer slots = Guest::Get<sead::Buffer>(groupPtr + offsetof(ASListSlotGroup, slots));
            const uint32_t slotsPtr = slots.data.getLE();
            const int32_t slotCount = slots.size.getLE();
            if (slotsPtr == 0) {
                continue;
            }

            for (int32_t slotIdx = 0; slotIdx < slotCount; slotIdx++) {
                uint32_t slotPtr = slotsPtr + static_cast<uint32_t>(slotIdx) * sizeof(ASListSlot);
                ASListSlot slot = Guest::Get<ASListSlot>(slotPtr);
                if (slot.assignedCount.getLE() == 0 || slot.definitionPtr.getLE() == 0) {
                    continue;
                }

                sead::SafeString name = Guest::Get<sead::SafeString>(slot.definitionPtr.getLE() + offsetof(ASListDefinitionPartial, name));
                std::string animationName = ReadLikelyName(name.c_str.getLE());
                if (animationName.empty()) {
                    continue;
                }

                animations.emplace_back(static_cast<uint32_t>(groupIdx), static_cast<uint32_t>(slotIdx), slotPtr, std::move(animationName));
            }
        }

        return animations;
    }

    static uint32_t ResolveActorPhysicsController(const ActorWiiU& actor) {
        uint32_t actorPhysics = actor.actorPhysicsPtr.getLE();
        if (actorPhysics == 0)
            return 0;

        return Guest::Get<BEType<uint32_t>>(actorPhysics + 0x50).getLE();
    }

    static bool IsTrackedPlayerBody(uint32_t rigidBodyAddr) {
        if (rigidBodyAddr == 0)
            return false;

        PlayerBase player;
        if (!Player::TryRead(player))
            return false;

        if (rigidBodyAddr == player.physicsTgtBodyPtr.getLE())
            return true;

        uint32_t mainBody = player.physicsMainBodyPtr.getLE();
        uint32_t controller = ResolveActorPhysicsController(player);
        if (controller != 0) {
            uint32_t controllerBody = Guest::Get<BEType<uint32_t>>(controller + 0x04).getLE();
            if (controllerBody != 0) {
                mainBody = controllerBody;
            }
        }

        return rigidBodyAddr == mainBody;
    }

private:
    static std::string ReadLikelyName(uint32_t namePtr) {
        if (namePtr == 0) {
            return {};
        }

        std::string name;
        name.reserve(64);

        for (uint32_t i = 0; i < 64; i++) {
            char c = *Guest::HostPointer(namePtr + i);
            if (c == '\0') {
                break;
            }

            bool validChar = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
            if (!validChar) {
                return {};
            }

            name.push_back(c);
        }

        return name;
    }
};
