#pragma once

#include "pch.h"


namespace RenderUtils {
    inline std::pair<glm::quat, glm::quat> swingTwistY(const glm::quat& q) {
        const glm::vec3 yAxis(0.0f, 1.0f, 0.0f);
        const glm::vec3 r(q.x, q.y, q.z);
        const float dot = glm::dot(r, yAxis);
        const glm::vec3 proj = yAxis * dot;
        const glm::quat twist = glm::normalize(glm::quat(q.w, proj.x, proj.y, proj.z));
        const glm::quat swing = q * glm::conjugate(twist);
        return { swing, twist };
    }

    struct UvTransform {
        float offsetX = 0.0f;
        float offsetY = 0.0f;
        float scaleX = 1.0f;
        float scaleY = 1.0f;
    };

    inline constexpr float RecordingRenderAspectRatio = 16.0f / 9.0f;
    inline constexpr float RecordingRenderAspectRatioTolerance = 0.01f;

    // the fixed 854x480 capture texture ScreenAppCamera::calledFromM159 (0x02F47A0C) allocates
    inline constexpr float CapturedImageAspectRatio = 854.0f / 480.0f;

    inline bool ShouldAdjustForRecordingRenderAspect(std::optional<float> renderAspectRatio) {
        return renderAspectRatio.has_value() && std::isfinite(renderAspectRatio.value()) && fabsf(renderAspectRatio.value() - RecordingRenderAspectRatio) <= RecordingRenderAspectRatioTolerance;
    }

    inline std::optional<float> SanitizeGameFovY(std::optional<float> fovY) {
        if (!fovY.has_value() || !std::isfinite(fovY.value()) || fovY.value() <= 0.0f || fovY.value() >= 3.13f) {
            return std::nullopt;
        }

        return fovY;
    }

    inline XrFovf CreateSymmetricFov(float fovY, float aspectRatio) {
        const float halfVertical = fovY * 0.5f;
        const float halfHorizontal = atanf(tanf(halfVertical) * aspectRatio);

        return XrFovf {
            .angleLeft = -halfHorizontal,
            .angleRight = halfHorizontal,
            .angleUp = halfVertical,
            .angleDown = -halfVertical,
        };
    }

    inline XrFovf ExpandFovToAspect(const XrFovf& fov, float targetAspectRatio) {
        const float left = tanf(fov.angleLeft);
        const float right = tanf(fov.angleRight);
        const float down = tanf(fov.angleDown);
        const float up = tanf(fov.angleUp);

        const float width = right - left;
        const float height = up - down;
        if (!std::isfinite(width) || !std::isfinite(height) || width <= 0.0f || height <= 0.0f || !std::isfinite(targetAspectRatio) || targetAspectRatio <= 0.0f) {
            return fov;
        }

        const float centerX = (left + right) * 0.5f;
        const float centerY = (up + down) * 0.5f;
        const float currentAspectRatio = width / height;

        float expandedWidth = width;
        float expandedHeight = height;
        if (currentAspectRatio < targetAspectRatio) {
            expandedWidth = height * targetAspectRatio;
        }
        else {
            expandedHeight = width / targetAspectRatio;
        }

        const float expandedLeft = centerX - expandedWidth * 0.5f;
        const float expandedRight = centerX + expandedWidth * 0.5f;
        const float expandedDown = centerY - expandedHeight * 0.5f;
        const float expandedUp = centerY + expandedHeight * 0.5f;

        return XrFovf {
            .angleLeft = atanf(expandedLeft),
            .angleRight = atanf(expandedRight),
            .angleUp = atanf(expandedUp),
            .angleDown = atanf(expandedDown),
        };
    }

    inline UvTransform CalculateSubFovUvTransform(const XrFovf& sourceFov, const XrFovf& targetFov) {
        const float srcLeft = tanf(sourceFov.angleLeft);
        const float srcRight = tanf(sourceFov.angleRight);
        const float srcTop = tanf(sourceFov.angleUp);
        const float srcBottom = tanf(sourceFov.angleDown);

        const float dstLeft = tanf(targetFov.angleLeft);
        const float dstRight = tanf(targetFov.angleRight);
        const float dstTop = tanf(targetFov.angleUp);
        const float dstBottom = tanf(targetFov.angleDown);

        const float srcWidth = srcRight - srcLeft;
        const float srcHeight = srcBottom - srcTop;
        const float dstWidth = dstRight - dstLeft;
        const float dstHeight = dstBottom - dstTop;

        UvTransform transform = {};
        if (!std::isfinite(srcWidth) || !std::isfinite(srcHeight) || srcWidth == 0.0f || srcHeight == 0.0f || !std::isfinite(dstWidth) || !std::isfinite(dstHeight)) {
            return transform;
        }

        transform.scaleX = dstWidth / srcWidth;
        transform.offsetX = (dstLeft - srcLeft) / srcWidth;
        transform.scaleY = dstHeight / srcHeight;
        transform.offsetY = (dstTop - srcTop) / srcHeight;
        return transform;
    }

    inline std::optional<XrFovf> GetRenderFov(const std::optional<XrFovf>& rawFov, std::optional<float> renderAspectRatio) {
        if (!rawFov.has_value()) {
            return std::nullopt;
        }

        if (!ShouldAdjustForRecordingRenderAspect(renderAspectRatio)) {
            return rawFov;
        }

        return ExpandFovToAspect(rawFov.value(), renderAspectRatio.value());
    }

    inline UvTransform GetSubFovUvTransform(const std::optional<XrFovf>& sourceFov, const std::optional<XrFovf>& targetFov) {
        if (!sourceFov.has_value() || !targetFov.has_value()) {
            return {};
        }

        return CalculateSubFovUvTransform(sourceFov.value(), targetFov.value());
    }

    inline UvTransform GetPresentationUvTransform(const std::optional<XrFovf>& rawFov, std::optional<float> renderAspectRatio) {
        return GetSubFovUvTransform(GetRenderFov(rawFov, renderAspectRatio), rawFov);
    }

    inline data_VRProjectionMatrixOut CalculateFOVAndOffset(const XrFovf& viewFOV) {
        const float left = tanf(viewFOV.angleLeft);
        const float right = tanf(viewFOV.angleRight);
        const float down = tanf(viewFOV.angleDown);
        const float up = tanf(viewFOV.angleUp);

        const float width = right - left;
        const float height = up - down;

        data_VRProjectionMatrixOut ret = {};
        if (!std::isfinite(width) || !std::isfinite(height) || width <= 0.0f || height <= 0.0f) {
            return ret;
        }

        ret.aspectRatio = width / height;
        ret.fovY = 2.0f * atanf(height * 0.5f);
        ret.offsetX = (left + right) / (2.0f * width);
        ret.offsetY = (up + down) / (2.0f * height);

        return ret;
    }

    // 0,0 is the top left, 1,1 the bottom right
    inline std::array<float, 2> ProjectDirectionToPicture(const XrFovf& fov, const glm::fvec3& direction) {
        const float left = tanf(fov.angleLeft);
        const float right = tanf(fov.angleRight);
        const float down = tanf(fov.angleDown);
        const float up = tanf(fov.angleUp);
        const glm::fvec3 ahead = direction.z < -0.01f ? direction : glm::fvec3(0.0f, 0.0f, -1.0f);
        const float x = ahead.x / -ahead.z;
        const float y = ahead.y / -ahead.z;
        return { std::clamp((x - left) / (right - left), 0.0f, 1.0f), std::clamp((up - y) / (up - down), 0.0f, 1.0f) };
    }

    inline glm::mat4 CalculateProjectionMatrix(float nearZ, float farZ, const XrFovf& fov) {
        float l = tanf(fov.angleLeft) * nearZ;
        float r = tanf(fov.angleRight) * nearZ;
        float b = tanf(fov.angleDown) * nearZ;
        float t = tanf(fov.angleUp) * nearZ;

        float invW = 1.0f / (r - l);
        float invH = 1.0f / (t - b);
        float invD = 1.0f / (farZ - nearZ);

        glm::mat4 dst = {};
        dst[0][0] = 2.0f * nearZ * invW;
        dst[1][1] = 2.0f * nearZ * invH;
        dst[0][2] = (r + l) * invW;
        dst[1][2] = (t + b) * invH;
        dst[2][2] = -(farZ + nearZ) * invD;
        dst[2][3] = -(2.0f * farZ * nearZ) * invD;
        dst[3][2] = -1.0f;
        dst[3][3] = 0.0f;

        return dst;
    }

    // sead::PerspectiveProjection's deviceZScale/deviceZOffset folded into a projection matrix
    inline glm::mat4 ApplyDeviceZTransform(glm::mat4 projection, float zScale, float zOffset) {
        projection[2][0] *= zScale;
        projection[2][1] *= zScale;
        projection[2][2] = (projection[2][2] + projection[3][2] * zOffset) * zScale;
        projection[2][3] = projection[2][3] * zScale + projection[3][3] * zOffset;
        return projection;
    }
}
