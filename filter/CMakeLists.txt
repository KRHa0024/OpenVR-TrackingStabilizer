add_library(ots_filter STATIC TrackingFilter.cpp TrackingFilter.h WaistModel.cpp WaistModel.h HandModel.cpp HandModel.h
            ImuOrientation.cpp ImuOrientation.h PoseMath.h)
target_include_directories(ots_filter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(ots_filter SYSTEM PUBLIC ${PROJECT_SOURCE_DIR}/external/openvr/headers)
target_link_libraries(ots_filter PUBLIC ots_common)
