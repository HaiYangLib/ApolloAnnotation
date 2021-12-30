add_library(timer_component_example SHARED
        timer_component_example.cc
        ${PROJECT_SOURCE_DIR}/test/proto/examples.pb.cc)

target_link_libraries(timer_component_example cyber_mine)
