# ImGui headers at the commit Cemu pins in its dependencies/imgui. Cemu links the core, src/CMakeLists.txt checks
# that the two are the same.
add_library(BetterVR_ImGui INTERFACE)
target_include_directories(BetterVR_ImGui INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/imgui)

add_library(BetterVR_ImPlot STATIC
    implot/implot.cpp
    implot/implot_items.cpp
)
target_include_directories(BetterVR_ImPlot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/implot)
target_link_libraries(BetterVR_ImPlot PUBLIC BetterVR_ImGui)

add_library(BetterVR_ImGuiVulkan STATIC imgui_impl_vulkan.cpp)
target_compile_definitions(BetterVR_ImGuiVulkan PUBLIC IMGUI_IMPL_VULKAN_NO_PROTOTYPES)
target_include_directories(BetterVR_ImGuiVulkan PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(BetterVR_ImGuiVulkan PUBLIC BetterVR_ImGui)

add_subdirectory(openxr)

set_target_properties(BetterVR_ImPlot BetterVR_ImGuiVulkan PROPERTIES FOLDER "dependencies")
