add_executable(test_talk talker.cc proto/examples.pb.cc)
target_link_libraries(test_talk cyber_mine)

add_executable(test_listen listener.cc proto/examples.pb.cc)
target_link_libraries(test_listen cyber_mine)

add_subdirectory(timer_component_example)
add_subdirectory(common_component_example)
