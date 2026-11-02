cmake_minimum_required(VERSION 3.20)
project(OpenVRTrackingStabilizer VERSION 0.1.0 LANGUAGES C CXX)

if(NOT WIN32 OR NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
  message(FATAL_ERROR "OpenVR-TrackingStabilizer targets Windows x64 only")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Static CRT so the driver DLL has no runtime dependency inside vrserver.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# Must match kDriverName in common/OtsConstants.h and the manifest "name".
set(OTS_DRIVER_NAME "trackingstabilizer")

add_subdirectory(external/minhook EXCLUDE_FROM_ALL)

# UI stack for ots-overlay: RmlUi (HTML/CSS-like UI) with FreeType text and lunasvg icons, all static.
set(CMAKE_POLICY_DEFAULT_CMP0091 NEW) # let the static CRT setting above reach the third-party projects
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
foreach(dep ZLIB BZIP2 PNG HARFBUZZ BROTLI)
  set(FT_DISABLE_${dep} ON CACHE BOOL "" FORCE)
endforeach()
add_subdirectory(external/freetype EXCLUDE_FROM_ALL)
target_compile_options(freetype PRIVATE /utf-8 /W0)
add_library(Freetype::Freetype ALIAS freetype)
set(LUNASVG_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(external/lunasvg EXCLUDE_FROM_ALL)
# The targets above stand in for find_package(); stop RmlUi from finding system copies instead.
set(CMAKE_DISABLE_FIND_PACKAGE_Freetype ON)
set(CMAKE_DISABLE_FIND_PACKAGE_lunasvg ON)
set(RMLUI_SVG_PLUGIN ON CACHE BOOL "" FORCE)
set(RMLUI_SAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(external/RmlUi EXCLUDE_FROM_ALL)

add_library(ots_common INTERFACE)
target_include_directories(ots_common INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/common)
target_compile_definitions(ots_common INTERFACE
  NOMINMAX WIN32_LEAN_AND_MEAN _CRT_SECURE_NO_WARNINGS)
if(MSVC)
  target_compile_options(ots_common INTERFACE /W4 /utf-8 /permissive-)
endif()

add_subdirectory(filter)
add_subdirectory(driver)
add_subdirectory(tools/ots-cli)
add_subdirectory(tools/ots-replay)
add_subdirectory(tools/ots-overlay)
enable_testing()
add_subdirectory(tests)
