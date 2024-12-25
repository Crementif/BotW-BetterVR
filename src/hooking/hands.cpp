#include "../instance.h"
#include "cemu_hooks.h"

struct ActorWiiU {
    uint32_t vtable;
    BEType<uint32_t> baseProcPtr;
    uint8_t unk_08[0xF4 - 0x08];
    uint32_t physicsMainBodyPtr; // 0xF4
    uint32_t physicsTgtBodyPtr; // 0xF8
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

    // ActorPhysics *physics is an interesting class, getPhysicsField60

    uint8_t unk_UNKNOWN[0x1C8];

    // todo: insert gap
    BEType<float> lodDrawDistanceMultiplier; // 0x490
};


static uint32_t stringToHash(const char* str) {
    uint32_t hash = 0;
    while (*str) {
        hash = (hash << 7) + *str++;
    }
    return hash;
}

std::mutex g_actorListMutex;
std::unordered_map<uint32_t, std::pair<std::string, uint32_t>> s_knownActors;

void CemuHooks::hook_UpdateActorList(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    std::scoped_lock lock(g_actorListMutex);

    // r7 holds actor list size
    // r5 holds current actor index
    // r6 holds current actor* list entry

    // clear actor list when reiterating actor list again
    if (hCPU->gpr[5] == 0) {
        s_knownActors.clear();
    }

    uint32_t actorLinkPtr = hCPU->gpr[6] + offsetof(ActorWiiU, baseProcPtr);
    uint32_t actorNamePtr = 0;
    readMemoryBE(actorLinkPtr, &actorNamePtr);
    if (actorNamePtr == 0)
        return;

    char* actorName = (char*)s_memoryBaseAddress + actorNamePtr;

    if (actorName[0] != '\0') {
        // Log::print("Updating actor list [{}/{}] {:08x} - {}", hCPU->gpr[5], hCPU->gpr[7], hCPU->gpr[6], actorName);
        uint32_t actorId = hCPU->gpr[6] + stringToHash(actorName);
        s_knownActors.emplace(actorId, std::make_pair(actorName, hCPU->gpr[6]));
    }

    // if (strcmp(actorName, "Weapon_Sword_056") == 0) {
    //     // Log::print("Updating actor list [{}/{}] {:08x} - {}", hCPU->gpr[5], hCPU->gpr[7], hCPU->gpr[6], actorName);
    //     // float velocityY = 0.0f;
    //     // readMemoryBE(hCPU->gpr[6] + offsetof(ActorWiiU, velocity.y), &velocityY);
    //     // velocityY = velocityY * 1.5f;
    //     // writeMemoryBE(hCPU->gpr[6] + offsetof(ActorWiiU, velocity.y), &velocityY);
    //     s_currActorPtrs.emplace_back(hCPU->gpr[6]);
    // }
}

// ksys::phys::RigidBodyFromShape::create to create a RigidBody from a shape
// use Actor::getRigidBodyByName

std::unordered_map<uint32_t, std::pair<std::string, uint32_t>> s_alreadyAddedActors;

void CemuHooks::updateFrames() {
    auto& overlay = VRManager::instance().VK->m_imguiOverlay;

    std::scoped_lock lock(g_actorListMutex);

    if (overlay) {
        // remove actors in s_alreadyAddedActors that are no longer in s_knownActors
        for (const auto& hash : s_alreadyAddedActors | std::views::keys) {
            if (!s_knownActors.contains(hash)) {
                overlay->RemoveEntity(hash);
            }
        }

        s_alreadyAddedActors = s_knownActors;

        // find current player (GameROMPlayer)
        BEMatrix34 playerPos = {};
        for (const auto& [actorId, actorData] : s_knownActors) {
            if (actorData.first == "GameROMPlayer") {
                readMemory(actorData.second + offsetof(ActorWiiU, mtx), &playerPos);
                break;
            }
        }

        // add actors that aren't in the overlay already
        for (auto& [actorId, actorInfo] : s_knownActors) {
            uint32_t actorPtr = actorInfo.second;
            const std::string& actorName = actorInfo.first;

            BEMatrix34 mtx = {};
            BEMatrix34 homeMtx = {};
            BEVec3 velocity = {};
            BEVec3 angularVelocity = {};

            readMemory(actorPtr + offsetof(ActorWiiU, mtx), &mtx);
            readMemory(actorPtr + offsetof(ActorWiiU, homeMtx), &homeMtx);
            readMemory(actorPtr + offsetof(ActorWiiU, velocity), &velocity);
            readMemory(actorPtr + offsetof(ActorWiiU, angularVelocity), &angularVelocity);

            if (playerPos.pos_x.getLE() != 0.0f) {
                overlay->SetPriority(actorId, playerPos.DistanceSq(mtx));
            }

            overlay->AddOrUpdateEntity(actorId, actorName, "mtx", actorPtr + offsetof(ActorWiiU, mtx), mtx);
            overlay->AddOrUpdateEntity(actorId, actorName, "homeMtx", actorPtr + offsetof(ActorWiiU, homeMtx), homeMtx);

            // uint32_t physicsMtxPtr = 0;
            // if (readMemoryBE(actorPtr + offsetof(ActorWiiU, physicsMtxPtr), &physicsMtxPtr); physicsMtxPtr != 0) {
            //     BEMatrix34 physicsMtx = {};
            //     readMemory(physicsMtxPtr, &physicsMtx);
            //     overlay->AddOrUpdateEntity(actorId, actorName, "physicsMtx", actorPtr + offsetof(ActorWiiU, physicsMtxPtr), physicsMtx);
            // }
            overlay->AddOrUpdateEntity(actorId, actorName, "velocity", actorPtr + offsetof(ActorWiiU, velocity), velocity);
            overlay->AddOrUpdateEntity(actorId, actorName, "angularVelocity", actorPtr + offsetof(ActorWiiU, angularVelocity), angularVelocity);
            overlay->AddOrUpdateEntity(actorId, actorName, "scale", actorPtr + offsetof(ActorWiiU, scale), getMemory<BEVec3>(actorPtr + offsetof(ActorWiiU, scale)));
            // overlay->AddOrUpdateEntity(actorId, actorName, "previousPos", actorPtr + offsetof(ActorWiiU, previousPos), getMemory<BEVec3>(actorPtr + offsetof(ActorWiiU, previousPos)));
            // overlay->AddOrUpdateEntity(actorId, actorName, "previousPos2", actorPtr + offsetof(ActorWiiU, previousPos2), getMemory<BEVec3>(actorPtr + offsetof(ActorWiiU, previousPos2)));
            // overlay->AddOrUpdateEntity(actorId, actorName, "dispDistSq", actorPtr + offsetof(ActorWiiU, dispDistSq), getMemory<float>(actorPtr + offsetof(ActorWiiU, dispDistSq)));
            // overlay->AddOrUpdateEntity(actorId, actorName, "deleteDistSq", actorPtr + offsetof(ActorWiiU, deleteDistSq), getMemory<float>(actorPtr + offsetof(ActorWiiU, deleteDistSq)));
            // overlay->AddOrUpdateEntity(actorId, actorName, "loadDistP10", actorPtr + offsetof(ActorWiiU, loadDistP10), getMemory<float>(actorPtr + offsetof(ActorWiiU, loadDistP10)));
            overlay->AddOrUpdateEntity(actorId, actorName, "lodDrawDistanceMultiplier", actorPtr + offsetof(ActorWiiU, lodDrawDistanceMultiplier), getMemory<float>(actorPtr + offsetof(ActorWiiU, lodDrawDistanceMultiplier)));
        }

        // other systems might've added memory to the overlay, so hence this is a separate loop
        for (auto& entity : VRManager::instance().VK->m_imguiOverlay->m_entities | std::views::values) {
            for (auto& value : entity.values) {
                if (!value.frozen)
                    continue;

                // Log::print("Freezing value {} from {}...", value.value_name, entity.name);
                std::visit([&](auto&& arg) {
                    using T = std::decay_t<decltype(arg)>;
                    if constexpr (std::is_same_v<T, BEType<uint32_t>>) {
                        writeMemory(value.value_address, &arg);
                    }
                    else if constexpr (std::is_same_v<T, BEType<int32_t>>) {
                        writeMemory(value.value_address, &arg);
                    }
                    else if constexpr (std::is_same_v<T, BEType<float>>) {
                        writeMemory(value.value_address, &arg);
                    }
                    else if constexpr (std::is_same_v<T, BEVec3>) {
                        writeMemory(value.value_address, &arg);
                    }
                    else if constexpr (std::is_same_v<T, BEMatrix34>) {
                        writeMemory(value.value_address, &arg);
                    }
                }, value.value);
            }
        }
    }
}

void CemuHooks::hook_changeWeaponMtx(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    // r3 holds an actor pointer, I think?
    // r4 holds the bone name
    // r5 holds the matrix that is to be set

    uint32_t actorLinkPtr = hCPU->gpr[3] + offsetof(ActorWiiU, baseProcPtr);
    uint32_t actorNamePtr = 0;
    readMemoryBE(actorLinkPtr, &actorNamePtr);
    if (actorNamePtr == 0)
        return;
    char* actorName = (char*)s_memoryBaseAddress + actorNamePtr;

    // read bone name
    uint32_t boneNamePtr = hCPU->gpr[4];
    if (boneNamePtr == 0)
        return;
    char* boneName = (char*)s_memoryBaseAddress + boneNamePtr;

    // real logic
    bool isHeldByPlayer = strcmp(actorName, "GameROMPlayer") == 0;
    bool isLeftHandWeapon = strcmp(boneName, "Weapon_L") == 0;
    bool isRightHandWeapon = strcmp(boneName, "Weapon_R") == 0;
    if (actorName[0] != '\0' && boneName[0] != '\0' && isHeldByPlayer && (isLeftHandWeapon || isRightHandWeapon)) {
        BEMatrix34 mtx = {};
        readMemory(hCPU->gpr[5], &mtx);

        BEMatrix34 extraMtx = {};
        readMemory(hCPU->gpr[6], &extraMtx);

        Log::print("Changing weapon matrix for {} with the bone {}:", actorName, boneName);
        Log::print("  -> destMtx: {}", mtx);
        Log::print("  -> extraMtx: {}", extraMtx);

        auto& m_overlay = VRManager::instance().VK->m_imguiOverlay;
        if (m_overlay) {
            m_overlay->AddOrUpdateEntity(1337, "PlayerHeldWeapons", isLeftHandWeapon ? "left_mtx" : "right_mtx", hCPU->gpr[5], mtx);
            m_overlay->AddOrUpdateEntity(1337, "PlayerHeldWeapons", isLeftHandWeapon ? "left_extra_mtx" : "right_extra_mtx", hCPU->gpr[6], extraMtx);
            m_overlay->SetPriority(1337, -1.0f);

            // freeze the value so it doesn't get overwritten
            Entity& entity = m_overlay->m_entities[1337];
            for (auto& value : entity.values) {
                if (value.value_name == (isLeftHandWeapon ? "left_mtx" : "right_mtx") && value.frozen) {
                    mtx = std::get<BEMatrix34>(value.value);
                    writeMemory(hCPU->gpr[5], &mtx);
                }
                else if (value.value_name == (isLeftHandWeapon ? "left_extra_mtx" : "right_extra_mtx") && value.frozen) {
                    extraMtx = std::get<BEMatrix34>(value.value);
                    writeMemory(hCPU->gpr[6], &extraMtx);
                }
                else {
                    // move mtx slightly upwards
                    mtx.y_x = mtx.y_x.getLE() + 5.1f;
                    mtx.z_x = mtx.z_x.getLE() + 5.1f;
                    mtx.pos_y = mtx.pos_y.getLE() + 5.1f;
                    mtx.x_z = mtx.x_z.getLE() + 5.1f;

                    // [x_x=-0.005497217, y_x=0, z_x=0.9999848, pos_x=-840.4056] [x_y=0, x_y=1, z_y=0, pos_y=199.16814] [x_z=-0.9999848, y_z=0, z_z=-0.005497217, pos_z=1797.2269]
                    // move extraMtx slightly upwards
                    extraMtx.pos_x = extraMtx.y_x.getLE() + 5.1f;
                    extraMtx.pos_y = extraMtx.pos_y.getLE() + 5.1f;
                    extraMtx.pos_z = extraMtx.pos_z.getLE() + 5.1f;

                    writeMemory(hCPU->gpr[5], &mtx);
                    writeMemory(hCPU->gpr[6], &extraMtx);
                }
            }
        }
    }
}

void CemuHooks::hook_modifyHandModelAccessSearch(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (hCPU->gpr[3] == 0) {
        return;
    }

    // r3 holds the address of the string to search for
    const char* actorName = (const char*)(s_memoryBaseAddress + hCPU->gpr[3]);

    if (actorName != nullptr) {
        // Weapon_R is presumably his right hand bone name
        Log::print("Searching for model handle using {}", actorName);
    }
}

void CemuHooks::hook_CreateNewActor(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (VRManager::instance().XR->GetRenderer() == nullptr || VRManager::instance().XR->GetRenderer()->m_layer3D.GetStatus() == RND_Renderer::Layer3D::Status3D::UNINITIALIZED) {
        hCPU->gpr[3] = 0;
        return;
    }

    // test if controller is connected
    if (VRManager::instance().XR->m_input.controllers[0].select.currentState == XR_TRUE && VRManager::instance().XR->m_input.controllers[0].select.changedSinceLastSync == XR_TRUE) {
        Log::print("Trying to spawn new thing!");
        hCPU->gpr[3] = 1;
    }
    else if (VRManager::instance().XR->m_input.controllers[1].select.currentState == XR_TRUE && VRManager::instance().XR->m_input.controllers[1].select.changedSinceLastSync == XR_TRUE) {
        Log::print("Trying to spawn new thing!");
        hCPU->gpr[3] = 1;
    }
    else {
        hCPU->gpr[3] = 0;
    }
}
