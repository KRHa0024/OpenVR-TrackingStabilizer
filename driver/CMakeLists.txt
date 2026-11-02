# Recorder has no OpenVR dependency so tests can link it without vrserver.
add_library(ots_recorder STATIC src/Recorder.cpp src/Recorder.h)
target_include_directories(ots_recorder PUBLIC src)
target_link_libraries(ots_recorder PUBLIC ots_common)

add_library(driver_${OTS_DRIVER_NAME} SHARED
  src/DriverMain.cpp
  src/DriverLog.cpp src/DriverLog.h
  src/FilterManager.cpp src/FilterManager.h
  src/Hooks.cpp src/Hooks.h
  src/LogRing.cpp src/LogRing.h
  src/PipeServer.cpp src/PipeServer.h
  src/ServerProvider.cpp src/ServerProvider.h)
target_include_directories(driver_${OTS_DRIVER_NAME} SYSTEM PRIVATE
  ${PROJECT_SOURCE_DIR}/external/openvr/headers
  ${PROJECT_SOURCE_DIR}/external/minhook/include)
target_link_libraries(driver_${OTS_DRIVER_NAME} PRIVATE ots_recorder ots_filter minhook)
# Always emit a PDB: vrserver crash dumps are useless without it.
target_compile_options(driver_${OTS_DRIVER_NAME} PRIVATE /Zi)
target_link_options(driver_${OTS_DRIVER_NAME} PRIVATE /DEBUG /OPT:REF /OPT:ICF)

# Assemble a loadable SteamVR driver folder: build/package/<driver>/
set(OTS_PACKAGE_DIR ${CMAKE_BINARY_DIR}/package/${OTS_DRIVER_NAME})
add_custom_command(TARGET driver_${OTS_DRIVER_NAME} POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/package ${OTS_PACKAGE_DIR}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${OTS_PACKAGE_DIR}/bin/win64
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:driver_${OTS_DRIVER_NAME}> ${OTS_PACKAGE_DIR}/bin/win64/
  COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_PDB_FILE:driver_${OTS_DRIVER_NAME}> ${OTS_PACKAGE_DIR}/bin/win64/
  COMMENT "Packaging SteamVR driver into ${OTS_PACKAGE_DIR}")
