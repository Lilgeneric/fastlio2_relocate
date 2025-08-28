#!/bin/bash
sleep 5  # 等待 gazebo 启动完成
rosconsole set /gazebo ros.gazebo_plugins info
