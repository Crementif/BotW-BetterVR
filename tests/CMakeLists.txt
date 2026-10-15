add_executable(BetterVR_ButtonTests button_state_tests.cpp)
target_include_directories(BetterVR_ButtonTests PRIVATE "${CMAKE_SOURCE_DIR}/src/game/botw/input")
add_test(NAME BetterVR.ButtonState COMMAND BetterVR_ButtonTests)
