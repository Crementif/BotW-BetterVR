#pragma once

// has to match the push constants in present.frag and present_depth.frag
struct presentSettings {
    float renderWidth;
    float renderHeight;
    float swapchainWidth;
    float swapchainHeight;
    float uvOffsetX;
    float uvOffsetY;
    float uvScaleX;
    float uvScaleY;
    float customFadeAmount;
    float customFadeColorR;
    float customFadeColorG;
    float customFadeColorB;
    float isFadeActive;
    float srgbEncode;
};

// has to match the push constants in debug_draw.vert and debug_draw.frag
struct debugDrawLineSettings {
    glm::mat4 viewProjection;
    float targetWidth;
    float targetHeight;
    float depthBias;
    float xrayAlphaScale;
};

static_assert(sizeof(presentSettings) == 56, "presentSettings no longer matches its push constant block");
static_assert(sizeof(debugDrawLineSettings) == 80, "debugDrawLineSettings no longer matches its push constant block");

// has to match the push constants in world_guide.vert and world_guide.frag
struct worldGuideSettings {
    glm::mat4 viewProjection;
    float srgbEncode;
};
