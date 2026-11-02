add_executable(recorder_selftest recorder_selftest.cpp)
target_link_libraries(recorder_selftest PRIVATE ots_recorder)
target_include_directories(recorder_selftest SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/external/openvr/headers)

add_executable(filter_test filter_test.cpp)
target_link_libraries(filter_test PRIVATE ots_filter)

add_executable(body_model_test body_model_test.cpp)
target_link_libraries(body_model_test PRIVATE ots_filter)

add_executable(hand_model_test hand_model_test.cpp)
target_link_libraries(hand_model_test PRIVATE ots_filter)

add_executable(imu_orientation_test imu_orientation_test.cpp)
target_link_libraries(imu_orientation_test PRIVATE ots_filter)

# Loads the built driver DLL into a fake vrserver (pass the DLL path as the argument).
add_executable(driver_harness driver_harness.cpp)
target_include_directories(driver_harness SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/external/openvr/headers)
target_compile_definitions(driver_harness PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
target_link_libraries(driver_harness PRIVATE winmm)
add_dependencies(driver_harness driver_${OTS_DRIVER_NAME})

add_test(NAME filter COMMAND filter_test)
add_test(NAME body_model COMMAND body_model_test)
add_test(NAME hand_model COMMAND hand_model_test)
add_test(NAME imu_orientation COMMAND imu_orientation_test)
add_test(NAME driver_harness COMMAND driver_harness $<TARGET_FILE:driver_${OTS_DRIVER_NAME}>)
