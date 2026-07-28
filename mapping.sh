#! /bin/bash
source ~/.bashrc
DIR="$( cd "$( dirname "$0" )" && pwd )"
source $DIR/../../devel/setup.bash

# 定义一个函数来终止所有子进程
terminate_processes() {
    echo "正在停止所有驱动程序..."
    for pid in ${pids[@]}; do
        if kill -0 $pid 2>/dev/null; then
            kill -TERM $pid
            wait $pid  # 等待进程响应TERM信号
        fi
    done
}


# 用于存储子进程PID的数组
pids=()

roslaunch sfast_lio mapping_mid360.launch &
pids+=($!)
sleep 5

roslaunch livox_mapping livox_mapping.launch &
pids+=($!)

# 等待所有子进程结束
wait ${pids[@]}

echo "已关闭所有程序！"
