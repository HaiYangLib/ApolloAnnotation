export CYBER_PATH=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
echo "CYBER_PATH:${CYBER_PATH}"
binary_path="/home/hhy/source_dir/cyber_mine/out/bin"
cyber_tool_path="${CYBER_PATH}/tools"
#recorder_path="${cyber_tool_path}/cyber_recorder"
#monitor_path="${cyber_tool_path}/cyber_monitor"

launch_path="${CYBER_PATH}/tools/cyber_launch"
channel_path="${CYBER_PATH}/tools/cyber_channel"
node_path="${CYBER_PATH}/tools/cyber_node"
service_path="${CYBER_PATH}/tools/cyber_service"

#export PATH=${binary_path}:${recorder_path}:${monitor_path}:${launch_path}:${channel_path}:${node_path}:${service_path}:${qt_path}/bin:${visualizer_path}:${rosbag_to_record_path}:$PATH
export PATH=${binary_path}:${launch_path}:${channel_path}:${node_path}:${service_path}:${qt_path}/bin:$PATH

export CYBER_DOMAIN_ID=80
export CYBER_IP=127.0.0.1

export GLOG_log_dir=/tmp/cyber_log
export GLOG_alsologtostderr=1
export GLOG_colorlogtostderr=1
export GLOG_minloglevel=0

export cyber_trans_perf=0
export cyber_sched_perf=0

# for DEBUG log
#export GLOG_minloglevel=-1
#export GLOG_v=4

echo ${CYBER_PATH}/tools/cyber_tools_auto_complete.bash
source ${CYBER_PATH}/tools/cyber_tools_auto_complete.bash
