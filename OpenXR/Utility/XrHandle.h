// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

namespace xr {

    template <typename HandleType>
    class UniqueExtHandle {
        using PFN_DestroyFunction = XrResult(XRAPI_PTR*)(HandleType);

    public:
        UniqueExtHandle() = default;
        UniqueExtHandle(const UniqueExtHandle&) = delete;
        UniqueExtHandle(UniqueExtHandle&& other) noexcept {
            *this = std::move(other);
        }

        ~UniqueExtHandle() noexcept {
            Reset();
        }

        UniqueExtHandle& operator=(const UniqueExtHandle&) = delete;
        UniqueExtHandle& operator=(UniqueExtHandle&& other) noexcept {
            if (m_handle != other.m_handle || m_destroyer != other.m_destroyer) {
                Reset();

                m_handle = other.m_handle;
                m_destroyer = other.m_destroyer;

                other.m_handle = XR_NULL_HANDLE;
                other.m_destroyer = nullptr;
            }
            return *this;
        }

        operator bool() const noexcept {
            return m_handle != XR_NULL_HANDLE;
        }

        HandleType Get() const noexcept {
            return m_handle;
        }

        // Extension functions cannot be statically linked, so the creator must pass in the destroy function.
        HandleType* Put(PFN_DestroyFunction destroyFunction) noexcept {
            assert(destroyFunction != nullptr);
            Reset();
            m_destroyer = destroyFunction;
            return &m_handle;
        }

        void Reset() noexcept {
            if (m_handle != XR_NULL_HANDLE) {
                m_destroyer(m_handle);
                m_handle = XR_NULL_HANDLE;
            }

            m_destroyer = nullptr;
        }

    private:
        HandleType m_handle{XR_NULL_HANDLE};
        PFN_DestroyFunction m_destroyer{nullptr};
    };

    template <typename HandleType, XrResult(XRAPI_PTR* DestroyFunction)(HandleType)>
    class UniqueHandle : public UniqueExtHandle<HandleType> {
    public:
        HandleType* Put() noexcept {
            return UniqueExtHandle<HandleType>::Put(DestroyFunction);
        }
    };

    class ActionHandle : public UniqueHandle<XrAction, xrDestroyAction> {};
    class ActionSetHandle : public UniqueHandle<XrActionSet, xrDestroyActionSet> {};
    class InstanceHandle : public UniqueHandle<XrInstance, xrDestroyInstance> {};
    class SessionHandle : public UniqueHandle<XrSession, xrDestroySession> {};
    class SpaceHandle : public UniqueHandle<XrSpace, xrDestroySpace> {};
    class SwapchainHandle : public UniqueHandle<XrSwapchain, xrDestroySwapchain> {};
    class SpatialAnchorHandle : public UniqueExtHandle<XrSpatialAnchorMSFT> {};
} // namespace xr
