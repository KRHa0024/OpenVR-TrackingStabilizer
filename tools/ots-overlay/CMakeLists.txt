set(OPENVR_DIR ${PROJECT_SOURCE_DIR}/external/openvr)
set(RMLUI_DIR ${PROJECT_SOURCE_DIR}/external/RmlUi)

add_executable(ots-overlay WIN32
  main.cpp
  app.rc
  DesktopWindow.cpp DesktopWindow.h
  DriverClient.cpp DriverClient.h
  Gfx.cpp Gfx.h
  Json.h
  Settings.cpp Settings.h
  State.h
  UiApp.cpp UiApp.h
  UiElements.cpp UiElements.h
  UiText.cpp UiText.h
  VrModelOverlays.cpp VrModelOverlays.h
  VrSource.cpp VrSource.h
  ${RMLUI_DIR}/Backends/RmlUi_Renderer_DX11.cpp)
target_include_directories(ots-overlay PRIVATE ${RMLUI_DIR}/Backends)
target_include_directories(ots-overlay SYSTEM PRIVATE ${OPENVR_DIR}/headers)
target_compile_definitions(ots-overlay PRIVATE OTS_VERSION="${PROJECT_VERSION}" NUM_MSAA_SAMPLES=4)
target_link_libraries(ots-overlay PRIVATE ots_common RmlUi::RmlUi ${OPENVR_DIR}/lib/win64/openvr_api.lib
  d3d11 dxgi d3dcompiler dxguid windowscodecs shell32)
# Console-less GUI app that still uses main(); a console is attached when started from one.
set_target_properties(ots-overlay PROPERTIES LINK_FLAGS "/ENTRY:mainCRTStartup")
# RmlUi's DX11 renderer is third-party code: keep our warning level away from it.
set_source_files_properties(${RMLUI_DIR}/Backends/RmlUi_Renderer_DX11.cpp PROPERTIES COMPILE_OPTIONS "/W0")

# The overlay runs from a folder holding the exe, openvr_api.dll, the manifest and the ui folder.
add_custom_command(TARGET ots-overlay POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENVR_DIR}/bin/win64/openvr_api.dll $<TARGET_FILE_DIR:ots-overlay>
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/manifest.vrmanifest $<TARGET_FILE_DIR:ots-overlay>
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/ui $<TARGET_FILE_DIR:ots-overlay>/ui)
