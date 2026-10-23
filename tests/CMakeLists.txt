add_executable(BetterVR_ButtonTests button_state_tests.cpp)
target_include_directories(BetterVR_ButtonTests PRIVATE "${CMAKE_SOURCE_DIR}/src/game/botw/input")
add_test(NAME BetterVR.ButtonState COMMAND BetterVR_ButtonTests)

find_package(glm CONFIG REQUIRED)
add_executable(BetterVR_SkeletonTests skeleton_tests.cpp)
target_link_libraries(BetterVR_SkeletonTests PRIVATE glm::glm)
add_test(NAME BetterVR.Skeleton COMMAND BetterVR_SkeletonTests)
