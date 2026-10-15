#pragma once
#include "vkroots.h"
#include "utils/platform.h"

template <>
struct fmt::formatter<VkResult> : fmt::formatter<std::string_view> {
    auto format(const VkResult format, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{} ({})", std::to_underlying(format), vkroots::helpers::enumString(format));
    }
};

template <>
struct fmt::formatter<XrResult> : fmt::formatter<std::string_view> {
    auto format(const XrResult format, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{}", std::to_underlying(format));
    }
};

template <>
struct fmt::formatter<VkFormat> : fmt::formatter<std::string_view> {
    auto format(const VkFormat format, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{} ({})", std::to_underlying(format), vkroots::helpers::enumString(format));
    }
};

template <>
struct fmt::formatter<glm::fmat3> : fmt::formatter<std::string_view> {
    auto format(const glm::fmat3& mtx, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{}", glm::to_string(mtx));
    }
};

template <>
struct fmt::formatter<glm::fmat4> : fmt::formatter<std::string_view> {
    auto format(const glm::fmat4& mtx, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{}", glm::to_string(mtx));
    }
};

template <>
struct fmt::formatter<glm::fmat3x4> : fmt::formatter<std::string_view> {
    auto format(const glm::fmat3x4& mtx, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{}", glm::to_string(mtx));
    }
};

template <>
struct fmt::formatter<glm::mat4x3> : fmt::formatter<std::string_view> {
    auto format(const glm::mat4x3& mtx, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{}", glm::to_string(mtx));
    }
};



template <>
struct fmt::formatter<glm::fvec2> : fmt::formatter<std::string_view> {
    auto format(const glm::fvec2& vec, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{}", glm::to_string(vec));
    }
};

template <>
struct fmt::formatter<glm::fvec3> : fmt::formatter<std::string_view> {
    auto format(const glm::fvec3& vec, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{}", glm::to_string(vec));
    }
};

template <>
struct fmt::formatter<glm::fvec4> : fmt::formatter<std::string_view> {
    auto format(const glm::fvec4& vec, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{}", glm::to_string(vec));
    }
};

template <>
struct fmt::formatter<glm::fquat> : fmt::formatter<std::string_view> {
    auto format(const glm::fquat& quat, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "[w={:.1f}, x={:.1f}, y={:.1f}, z={:.1f}] (euler: x={:.1f}, y={:.1f}, z={:.1f})", quat.w, quat.x, quat.y, quat.z, glm::degrees(glm::eulerAngles(quat)).x, glm::degrees(glm::eulerAngles(quat)).y, glm::degrees(glm::eulerAngles(quat)).z);
    }
};

template <>
struct fmt::formatter<BEVec3> : fmt::formatter<std::string_view> {
    auto format(const BEVec3& vec, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{}", glm::to_string(vec.getLE()));
    }
};

template <>
struct fmt::formatter<BEMatrix34> : fmt::formatter<std::string_view> {
    auto format(const BEMatrix34& mtx, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "[x_x={:.1f}, y_x={:.1f}, z_x={:.1f}, pos_x={:.1f}] [x_y={:.1f}, x_y={:.1f}, z_y={:.1f}, pos_y={:.1f}] [x_z={:.1f}, y_z={:.1f}, z_z={:.1f}, pos_z={:.1f}]",
            mtx.x_x.getLE(), mtx.y_x.getLE(), mtx.z_x.getLE(), mtx.pos_x.getLE(),
            mtx.x_y.getLE(), mtx.y_y.getLE(), mtx.z_y.getLE(), mtx.pos_y.getLE(),
            mtx.x_z.getLE(), mtx.y_z.getLE(), mtx.z_z.getLE(), mtx.pos_z.getLE()
        );
    }
};

template <>
struct fmt::formatter<BEMatrix44> : fmt::formatter<std::string_view> {
    auto format(const BEMatrix44& mtx, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{}", glm::to_string(mtx.getLE()));
    }
};

template <>
struct fmt::formatter<BESeadProjection> : fmt::formatter<std::string_view> {
    auto format(const BESeadProjection& proj, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(),
            "vtable = {:08X}, dirty = {}, deviceDirty = {}, matrix = {}, deviceMatrix = {} devicePosture = {}, deviceZOffset = {}, deviceZScale = {}",
            proj.__vftable.getLE(), proj.dirty.getLE(), proj.deviceDirty.getLE(), proj.matrix, proj.deviceMatrix, proj.devicePosture.getLE(), proj.deviceZOffset.getLE(), proj.deviceZScale.getLE()
        );
    }
};

template <>
struct fmt::formatter<BESeadPerspectiveProjection> : fmt::formatter<std::string_view> {
    auto format(const BESeadPerspectiveProjection& proj, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(),
            "near = {}, far = {}, angle = {}, fovySin = {}, fovyCos = {}, fovyTan = {}, aspect = {}, offsetX = {}, offsetY = {}\r\ndirty = {}, deviceDirty = {}, matrix = {}, deviceMatrix = {} devicePosture = {}, deviceZOffset = {}, deviceZScale = {}",
            proj.zNear.getLE(), proj.zFar.getLE(), proj.fovYRadiansOrAngle.getLE(), proj.fovySin.getLE(), proj.fovyCos.getLE(), proj.fovyTan.getLE(), proj.aspect.getLE(), proj.offset.x.getLE(), proj.offset.y.getLE(),
            proj.dirty.getLE(), proj.deviceDirty.getLE(), proj.matrix, proj.deviceMatrix, proj.devicePosture.getLE(), proj.deviceZOffset.getLE(), proj.deviceZScale.getLE()
        );
    }
};

template <>
struct fmt::formatter<BESeadCamera> : fmt::formatter<std::string_view> {
    auto format(const BESeadCamera& cam, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "mtx = {}, vtbl = {:08X}", cam.mtx, cam.__vftable.getLE());
    }
};

template <>
struct fmt::formatter<BESeadLookAtCamera> : fmt::formatter<std::string_view> {
    auto format(const BESeadLookAtCamera& cam, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "mtx = {}, pos = {}, at = {}, up = {}", cam.mtx, cam.pos, cam.at, cam.up);
    }
};

template <>
struct fmt::formatter<LookAtMatrix> : fmt::formatter<std::string_view> {
    auto format(const LookAtMatrix& cam, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "pos = {}, target = {}, up = {}, unknown = {}, zNear = {}, zFar = {}", cam.pos, cam.target, cam.up, cam.unknown, cam.zNear.getLE(), cam.zFar.getLE());
    }
};


static std::string FormatDistance(float distance) {
    float distanceInches = distance * 39.3700787f;
    int32_t distanceFeet = std::floor(distanceInches / 12.0f);
    distanceInches -= distanceFeet * 12.0f;
    return fmt::format("{:.02f}m / {}\' {:.02f}\"", distance, distanceFeet, distanceInches);
}

enum class LogType {
    // verbose logging types
    RENDERING,
    INTEROP,
    CONTROLS,
    PPC,
    XR_DEBUGUTILS,
    ARROW_SHOT_CAPTURE,
    ROOMSCALE,

    // generic types
    INFO,
    WARNING,
    ERROR,
    VERBOSE
};

using enum LogType;
ENABLE_BITMASK_OPERATORS(LogType);

class Log {
public:
    static constexpr size_t kLogTypeCount = (size_t)VERBOSE + 1;

    Log();
    ~Log();

    template <LogType L>
    static inline bool consteval isLogTypeEnabled() {
        return true;
    }

    static bool IsCategoryEnabled(LogType type) {
        return s_categoryEnabled[(size_t)type].load(std::memory_order_relaxed);
    }

    static void SetCategoryEnabled(LogType type, bool enabled) {
        s_categoryEnabled[(size_t)type].store(enabled, std::memory_order_relaxed);
    }

    template <LogType L>
    static void SetCategory(bool enabled) {
        SetCategoryEnabled(L, enabled);
    }

    static void SetShowTimestamps(bool enabled);
    static void SetShowThreadIds(bool enabled);

    template <LogType L>
    static inline void print(const char* message) {
        if constexpr (!isLogTypeEnabled<L>()) {
            return;
        }
        if constexpr (L != INFO && L != WARNING && L != ERROR) {
            if (!IsCategoryEnabled(L)) {
                return;
            }
        }
        submit(L, std::string_view(message));
        // flush after errors since crash might be imminent
        if constexpr (L == ERROR) {
            Flush();
        }
    }

    template <LogType L, class... Args>
    static inline void print(const char* format, Args&&... args) {
        if constexpr (!isLogTypeEnabled<L>()) {
            return;
        }
        if constexpr (L != INFO && L != WARNING && L != ERROR) {
            if (!IsCategoryEnabled(L)) {
                return;
            }
        }
        s_scratch.clear();
        fmt::vformat_to(std::back_inserter(s_scratch), format, fmt::make_format_args(args...));
        submit(L, std::string_view(s_scratch));
        if constexpr (L == ERROR) {
            Flush();
        }
    }

    static void printTimeElapsed(const char* message_prefix, uint64_t startTimeStamp);
    static void Flush();
    // before the first message, BETTERVR_LOG_PATH still wins
    static void SetFilePath(const std::filesystem::path& path);

    // ref counted shutdown of the logger
    static void OnInstanceCreated();
    static void OnInstanceDestroyed();

private:
    static void submit(LogType type, std::string_view message);

    inline static thread_local std::string s_scratch;

    inline static constinit std::array<std::atomic_bool, kLogTypeCount> s_categoryEnabled = {};
};

static void checkXRResult(const XrResult result, const char* errorMessage) {
    if (XR_FAILED(result)) {
        if (errorMessage == nullptr) {
            Log::print<ERROR>("An unknown error (result was {}) has occurred!", result);
            Log::Flush();
#ifdef _DEBUG
            BETTERVR_DEBUG_BREAK();
#endif
            Platform::ShowFatalError("An error occurred!", fmt::format("An unknown error {} has occurred which caused a fatal crash!", result).c_str());
            throw std::runtime_error("Unidentified error occurred!");
        }
        else {
            Log::print<ERROR>("Error {}: {}", result, errorMessage);
            Log::Flush();
#ifdef _DEBUG
            BETTERVR_DEBUG_BREAK();
#endif
            Platform::ShowFatalError("A fatal error occurred!", errorMessage);
            throw std::runtime_error(errorMessage);
        }
    }
}

static void checkVkResult(const VkResult result, const char* errorMessage) {
    if (result != VK_SUCCESS) {
        if (errorMessage == nullptr) {
            Log::print<ERROR>("An unknown error (result was {}) has occurred!", (std::underlying_type_t<VkResult>)result);
            Log::Flush();
#ifdef _DEBUG
            BETTERVR_DEBUG_BREAK();
#endif
            Platform::ShowFatalError("A fatal error occurred!", fmt::format("An unknown error {} has occurred which caused a fatal crash!", (std::underlying_type_t<VkResult>)result).c_str());
            throw std::runtime_error("Unidentified error occurred!");
        }
        else {
            Log::print<ERROR>("Error {}: {}", (std::underlying_type_t<VkResult>)result, errorMessage);
            Log::Flush();
#ifdef _DEBUG
            BETTERVR_DEBUG_BREAK();
#endif
            Platform::ShowFatalError("A fatal error occurred!", errorMessage);
            throw std::runtime_error(errorMessage);
        }
    }
}

static void checkAssert(const bool assert, const char* errorMessage) {
    if (!assert) {
        if (errorMessage == nullptr) {
            Log::print<ERROR>("Something unexpected happened that prevents further execution!");
            Log::Flush();
#ifdef _DEBUG
            BETTERVR_DEBUG_BREAK();
#endif
            Platform::ShowFatalError("A fatal error occurred!", "Something unexpected happened that prevents further execution!");
            throw std::runtime_error("Unexpected assertion occurred!");
        }
        else {
            Log::print<ERROR>("{}", errorMessage);
            Log::Flush();
#ifdef _DEBUG
            BETTERVR_DEBUG_BREAK();
#endif
            Platform::ShowFatalError("A fatal error occurred!", errorMessage);
            throw std::runtime_error(errorMessage);
        }
    }
}
