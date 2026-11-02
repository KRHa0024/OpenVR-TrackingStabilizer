add_executable(ots-cli main.cpp Speech.cpp Speech.h)
target_link_libraries(ots-cli PRIVATE ots_common sapi ole32)
