add_library(common_component_example SHARED
        common_component_example.cc
        ${PROJECT_SOURCE_DIR}/test/proto/examples.pb.cc)

target_link_libraries(common_component_example cyber_mine)