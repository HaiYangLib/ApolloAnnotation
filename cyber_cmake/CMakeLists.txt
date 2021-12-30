cmake_minimum_required(VERSION 3.5)

project(cyber_mine)

# 允许使用汇编
enable_language(C ASM)
set(CMAKE_ASM_CREATE_SHARED_LIBRARY true)


# # 设置输出路径
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/out/lib)
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/out/bin)


## clang-8
#set(CMAKE_C_COMPILER /usr/bin/clang-8)
#set(CMAKE_CXX_COMPILER /usr/bin/clang++-8)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O0 -DNDEBUG" CACHE STRING "" FORCE)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
#message("CXX_FLAGS:${CMAKE_CXX_FLAGS}")

# gcc
set(CMAKE_C_COMPILER /usr/bin/gcc)
set(CMAKE_CXX_COMPILER /usr/bin/g++)
set(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} -latomic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O0 -DNDEBUG" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

include_directories(${PROJECT_SOURCE_DIR})
# ======================cyber依赖的第三方库========================
set(poco_path ${PROJECT_SOURCE_DIR}/third_party/poco/poco-poco-1.11.1-release-c8-libstdcxx-install)
set(fast_cdr_path ${PROJECT_SOURCE_DIR}/third_party/fast_rtps/fast_cdr-v1.0.22-gcc-libstdcxx-install/usr/local)
set(fast_rtps_path ${PROJECT_SOURCE_DIR}/third_party/fast_rtps/fast_rtps-v1.5.0-gcc-libstdcxx-install/usr/local)
set(glog_path ${PROJECT_SOURCE_DIR}/third_party/glog/glog-v0.3.5-gcc-libstdcxx-install)
set(gflags_path ${PROJECT_SOURCE_DIR}/third_party/glog/gflag-v2.2.0-gcc-libstdcxx-install)
set(protobuf_path ${PROJECT_SOURCE_DIR}/third_party/protobuf/protobuf-v3.3.0-gcc-libstdcxx-install)
# poco
include_directories(${poco_path}/include)
link_directories(${poco_path}/lib)
# fast_cdr
include_directories(${fast_cdr_path}/include)
link_directories(${fast_cdr_path}/lib)
# fast_rtps
include_directories(${fast_rtps_path}/include)
link_directories(${fast_rtps_path}/lib)
# glog
include_directories(${glog_path}/include)
link_directories(${glog_path}/lib)
# gflag
include_directories(${gflags_path}/include)
link_directories(${gflags_path}/lib)
# protobuf
include_directories(${protobuf_path}/include)
link_directories(${protobuf_path}/lib)

set(cyber_source_path "${PROJECT_SOURCE_DIR}")
file(GLOB cyber_sources
        ${cyber_source_path}/cyber/*.h
        ${cyber_source_path}/cyber/*.cc

        ${cyber_source_path}/cyber/base/*.h
        ${cyber_source_path}/cyber/base/*.cc

        ${cyber_source_path}/cyber/blocker/*.h
        ${cyber_source_path}/cyber/blocker/*.cc

        ${cyber_source_path}/cyber/class_loader/*.h
        ${cyber_source_path}/cyber/class_loader/*.cc
        ${cyber_source_path}/cyber/class_loader/utility/*.h
        ${cyber_source_path}/cyber/class_loader/utility/*.cc

        ${cyber_source_path}/cyber/common/*.h
        ${cyber_source_path}/cyber/common/*.cc

        ${cyber_source_path}/cyber/component/*.h
        ${cyber_source_path}/cyber/component/*.cc

        ${cyber_source_path}/cyber/croutine/*.h
        ${cyber_source_path}/cyber/croutine/*.cc
        ${cyber_source_path}/cyber/croutine/detail/swap_x86_64.S
        ${cyber_source_path}/cyber/croutine/detail/*.h
        ${cyber_source_path}/cyber/croutine/detail/*.cc

        ${cyber_source_path}/cyber/data/*.h
        ${cyber_source_path}/cyber/data/*.cc
        ${cyber_source_path}/cyber/data/fusion/*.h
        ${cyber_source_path}/cyber/data/fusion/*.cc

        ${cyber_source_path}/cyber/event/*.h
        ${cyber_source_path}/cyber/event/*.cc

        ${cyber_source_path}/cyber/io/*.h
        ${cyber_source_path}/cyber/io/*.cc

        ${cyber_source_path}/cyber/logger/*.h
        ${cyber_source_path}/cyber/logger/*.cc

        ${cyber_source_path}/cyber/message/*.h
        ${cyber_source_path}/cyber/message/*.cc

        ${cyber_source_path}/cyber/node/*.h
        ${cyber_source_path}/cyber/node/*.cc

        ${cyber_source_path}/cyber/parameter/*.h
        ${cyber_source_path}/cyber/parameter/*.cc

        ${cyber_source_path}/cyber/proto/*.h
        ${cyber_source_path}/cyber/proto/*.cc

        ${cyber_source_path}/cyber/record/*.h
        ${cyber_source_path}/cyber/record/*.cc
        ${cyber_source_path}/cyber/record/file/*.h
        ${cyber_source_path}/cyber/record/file/*.cc

        ${cyber_source_path}/cyber/scheduler/*.h
        ${cyber_source_path}/cyber/scheduler/*.cc
        ${cyber_source_path}/cyber/scheduler/common/*.h
        ${cyber_source_path}/cyber/scheduler/common/*.cc
        ${cyber_source_path}/cyber/scheduler/policy/*.h
        ${cyber_source_path}/cyber/scheduler/policy/*.cc

        ${cyber_source_path}/cyber/service_discovery/*.h
        ${cyber_source_path}/cyber/service_discovery/*.cc
        ${cyber_source_path}/cyber/service_discovery/communication/*.h
        ${cyber_source_path}/cyber/service_discovery/communication/*.cc
        ${cyber_source_path}/cyber/service_discovery/container/*.h
        ${cyber_source_path}/cyber/service_discovery/container/*.cc
        ${cyber_source_path}/cyber/service_discovery/role/*.h
        ${cyber_source_path}/cyber/service_discovery/role/*.cc
        ${cyber_source_path}/cyber/service_discovery/specific_manager/*.h
        ${cyber_source_path}/cyber/service_discovery/specific_manager/*.cc

        ${cyber_source_path}/cyber/task/*.h
        ${cyber_source_path}/cyber/task/*.cc

        ${cyber_source_path}/cyber/time/*.h
        ${cyber_source_path}/cyber/time/*.cc

        ${cyber_source_path}/cyber/timer/*.h
        ${cyber_source_path}/cyber/timer/*.cc

        ${cyber_source_path}/cyber/transport/*.h
        ${cyber_source_path}/cyber/transport/*.cc
        ${cyber_source_path}/cyber/transport/common/*.h
        ${cyber_source_path}/cyber/transport/common/*.cc
        ${cyber_source_path}/cyber/transport/dispatcher/*.h
        ${cyber_source_path}/cyber/transport/dispatcher/*.cc
        ${cyber_source_path}/cyber/transport/message/*.h
        ${cyber_source_path}/cyber/transport/message/*.cc
        ${cyber_source_path}/cyber/transport/qos/*.h
        ${cyber_source_path}/cyber/transport/qos/*.cc
        ${cyber_source_path}/cyber/transport/receiver/*.h
        ${cyber_source_path}/cyber/transport/receiver/*.cc
        ${cyber_source_path}/cyber/transport/rtps/*.h
        ${cyber_source_path}/cyber/transport/rtps/*.cc
        ${cyber_source_path}/cyber/transport/shm/*.h
        ${cyber_source_path}/cyber/transport/shm/*.cc
        ${cyber_source_path}/cyber/transport/transceiver/*.h
        ${cyber_source_path}/cyber/transport/transceiver/*.cc
        ${cyber_source_path}/cyber/transport/transmitter/*.h
        ${cyber_source_path}/cyber/transport/transmitter/*.cc)

# ============================cyber_mine==================================
add_library(cyber_mine SHARED ${cyber_sources})
target_link_libraries(cyber_mine protobuf fastcdr fastrtps gflags glog PocoFoundation uuid atomic)
install(TARGETS cyber_mine DESTINATION lib)
# ===========================install======================================

file(GLOB cyber_h ${cyber_source_path}/cyber/*.h)
install(FILES ${cyber_h} DESTINATION include/cyber)

file(GLOB cyber_base_h ${cyber_source_path}/cyber/base/*.h)
install(FILES ${cyber_base_h} DESTINATION include/cyber/base)

file(GLOB cyber_blocker_h ${cyber_source_path}/cyber/blocker/*.h)
install(FILES ${cyber_blocker_h} DESTINATION include/cyber/blocker)

file(GLOB cyber_loader_h ${cyber_source_path}/cyber/class_loader/*.h)
install(FILES ${cyber_loader_h} DESTINATION include/cyber/class_loader)

file(GLOB cyber_utility_h ${cyber_source_path}/cyber/class_loader/utility/*.h)
install(FILES ${cyber_utility_h} DESTINATION include/cyber/class_loader/utility)

file(GLOB cyber_common_h ${cyber_source_path}/cyber/common/*.h)
install(FILES ${cyber_common_h} DESTINATION include/cyber/common)

file(GLOB cyber_component_h ${cyber_source_path}/cyber/component/*.h)
install(FILES ${cyber_component_h} DESTINATION include/cyber/component)

file(GLOB cyber_croutine_h ${cyber_source_path}/cyber/croutine/*.h)
install(FILES ${cyber_croutine_h} DESTINATION include/cyber/croutine)

file(GLOB cyber_detail_h ${cyber_source_path}/cyber/croutine/detail/*.h)
install(FILES ${cyber_detail_h} DESTINATION include/cyber/croutine/detail)

file(GLOB cyber_data_h ${cyber_source_path}/cyber/data/*.h)
install(FILES ${cyber_data_h} DESTINATION include/cyber/data)

file(GLOB cyber_fusion_h ${cyber_source_path}/cyber/data/fusion/*.h)
install(FILES ${cyber_fusion_h} DESTINATION include/cyber/data/fusion)

file(GLOB cyber_event_h ${cyber_source_path}/cyber/event/*.h)
install(FILES ${cyber_event_h} DESTINATION include/cyber/event)

file(GLOB cyber_io_h ${cyber_source_path}/cyber/io/*.h)
install(FILES ${cyber_io_h} DESTINATION include/cyber/io)

file(GLOB cyber_logger_h ${cyber_source_path}/cyber/logger/*.h)
install(FILES ${cyber_logger_h} DESTINATION include/cyber/logger)

file(GLOB cyber_message_h ${cyber_source_path}/cyber/message/*.h)
install(FILES ${cyber_message_h} DESTINATION include/cyber/message)

file(GLOB cyber_node_h ${cyber_source_path}/cyber/node/*.h)
install(FILES ${cyber_node_h} DESTINATION include/cyber/node)

file(GLOB cyber_parameter_h ${cyber_source_path}/cyber/parameter/*.h)
install(FILES ${cyber_parameter_h} DESTINATION include/cyber/parameter)

file(GLOB cyber_proto_h ${cyber_source_path}/cyber/proto/*.h)
install(FILES ${cyber_proto_h} DESTINATION include/cyber/proto)

file(GLOB cyber_record_h ${cyber_source_path}/cyber/record/*.h)
install(FILES ${cyber_record_h} DESTINATION include/cyber/record)

file(GLOB cyber_file_h ${cyber_source_path}/cyber/record/file/*.h)
install(FILES ${cyber_file_h} DESTINATION include/cyber/record/file)

file(GLOB cyber_scheduler_h ${cyber_source_path}/cyber/scheduler/*.h)
install(FILES ${cyber_scheduler_h} DESTINATION include/cyber/scheduler)

file(GLOB cyber_scheduler_common_h ${cyber_source_path}/cyber/scheduler/common/*.h)
install(FILES ${cyber_scheduler_common_h} DESTINATION include/cyber/scheduler/common)

file(GLOB cyber_policy_h ${cyber_source_path}/cyber/scheduler/policy/*.h)
install(FILES ${cyber_policy_h} DESTINATION include/cyber/scheduler/policy)

file(GLOB cyber_service_discovery_h ${cyber_source_path}/cyber/service_discovery/*.h)
install(FILES ${cyber_service_discovery_h} DESTINATION include/cyber/service_discovery)

file(GLOB cyber_communication_h ${cyber_source_path}/cyber/service_discovery/communication/*.h)
install(FILES ${cyber_communication_h} DESTINATION include/cyber/service_discovery/communication)

file(GLOB cyber_container_h ${cyber_source_path}/cyber/service_discovery/container/*.h)
install(FILES ${cyber_container_h} DESTINATION include/cyber/service_discovery/container)

file(GLOB cyber_role_h ${cyber_source_path}/cyber/service_discovery/role/*.h)
install(FILES ${cyber_role_h} DESTINATION include/cyber/service_discovery/role)

file(GLOB cyber_specific_manager_h ${cyber_source_path}/cyber/service_discovery/specific_manager/*.h)
install(FILES ${cyber_specific_manager_h} DESTINATION include/cyber/service_discovery/specific_manager)

file(GLOB cyber_task_h ${cyber_source_path}/cyber/task/*.h)
install(FILES ${cyber_task_h} DESTINATION include/cyber/task)

file(GLOB cyber_time_h ${cyber_source_path}/cyber/time/*.h)
install(FILES ${cyber_time_h} DESTINATION include/cyber/time)

file(GLOB cyber_timer_h ${cyber_source_path}/cyber/timer/*.h)
install(FILES ${cyber_timer_h} DESTINATION include/cyber/timer)

file(GLOB cyber_transport_h ${cyber_source_path}/cyber/transport/*.h)
install(FILES ${cyber_transport_h} DESTINATION include/cyber/transport)

file(GLOB cyber_transport_common_h ${cyber_source_path}/cyber/transport/common/*.h)
install(FILES ${cyber_transport_common_h} DESTINATION include/cyber/transport/common)

file(GLOB cyber_transport_dispatcher_h ${cyber_source_path}/cyber/transport/dispatcher/*.h)
install(FILES ${cyber_transport_dispatcher_h} DESTINATION include/cyber/transport/dispatcher)

file(GLOB cyber_transport_message_h ${cyber_source_path}/cyber/transport/message/*.h)
install(FILES ${cyber_transport_message_h} DESTINATION include/cyber/transport/message)

file(GLOB cyber_transport_qos_h ${cyber_source_path}/cyber/transport/qos/*.h)
install(FILES ${cyber_transport_qos_h} DESTINATION include/cyber/transport/qos)

file(GLOB cyber_transport_receiver_h ${cyber_source_path}/cyber/transport/receiver/*.h)
install(FILES ${cyber_transport_receiver_h} DESTINATION include/cyber/transport/receiver)

file(GLOB cyber_transport_rtps_h ${cyber_source_path}/cyber/transport/rtps/*.h)
install(FILES ${cyber_transport_rtps_h} DESTINATION include/cyber/transport/rtps)

file(GLOB cyber_transport_shm_h ${cyber_source_path}/cyber/transport/shm/*.h)
install(FILES ${cyber_transport_shm_h} DESTINATION include/cyber/transport/shm)

file(GLOB cyber_transport_transceiver_h ${cyber_source_path}/cyber/transport/transceiver/*.h)
install(FILES ${cyber_transport_transceiver_h} DESTINATION include/cyber/transport/transceiver)

file(GLOB cyber_transport_transmitter_h ${cyber_source_path}/cyber/transport/transmitter/*.h)
install(FILES ${cyber_transport_transmitter_h} DESTINATION include/cyber/transport/transmitter)

# ============================mainboard===================================
add_executable(mainboard
        ${cyber_source_path}/cyber/mainboard/mainboard.cc
        ${cyber_source_path}/cyber/mainboard/module_argument.cc
        ${cyber_source_path}/cyber/mainboard/module_controller.cc)
target_link_libraries(mainboard cyber_mine)
install(TARGETS mainboard DESTINATION bin)

# ============================cyber_monitor===============================
file(GLOB cyber_monitor_src
        ${cyber_source_path}/cyber/tools/cyber_monitor/*.h
        ${cyber_source_path}/cyber/tools/cyber_monitor/*.cc)
add_executable(cyber_monitor ${cyber_monitor_src})
target_link_libraries(cyber_monitor cyber_mine curses)
install(TARGETS cyber_monitor DESTINATION bin)
# ===========================cyber_recorder===============================
file(GLOB cyber_recorder_src
        ${cyber_source_path}/cyber/tools/cyber_recorder/*.h
        ${cyber_source_path}/cyber/tools/cyber_recorder/*.cc
        ${cyber_source_path}/cyber/tools/cyber_recorder/player/*.h
        ${cyber_source_path}/cyber/tools/cyber_recorder/player/*.cc)
add_executable(cyber_recorder ${cyber_recorder_src})
target_link_libraries(cyber_recorder cyber_mine)
install(TARGETS cyber_recorder DESTINATION bin)

# ===============================test=====================================
add_subdirectory(test)