# description

## 依赖

xacro 将`.xacro`文件解析为`.urdf.xml`文件

rviz2 可视化机器人系统

robot_state_publisher 发布机器人组件的相关tf信息

joint_state_broadcaster、joint_state_publisher_gui 发布机器人活动关节信息（非静态tf）

## 交互关系

以下仅列出相关节点中的关键通信连接

```shell
ros2 node info /rviz2
# /rviz2
#   Subscribers:
#     /robot_description: std_msgs/msg/String
#   Publishers:
#     /clicked_point: geometry_msgs/msg/PointStamped
#     /goal_pose: geometry_msgs/msg/PoseStamped
#     /initialpose: geometry_msgs/msg/PoseWithCovarianceStamped

ros2 node info /robot_state_publisher
# /robot_state_publisher
#   Subscribers:
#     /joint_states: sensor_msgs/msg/JointState
#   Publishers:
#     /robot_description: std_msgs/msg/String
#     /tf: tf2_msgs/msg/TFMessage
#     /tf_static: tf2_msgs/msg/TFMessage

ros2 node info /joint_state_publisher
# /joint_state_publisher
#   Subscribers:
#     /robot_description: std_msgs/msg/String
#   Publishers:
#     /joint_states: sensor_msgs/msg/JointState

```
首先， 解析好的机器人描述`PARAM: urdf`信息通过launch文件传入`NODE: /robot_state_publisher`节点，这些内容通过`TOPIC:  /robot_description`同步到另外两个节点；

`NODE: /joint_state_publisher`节点会发布`TOPIC:  /joint_states`描述活动关节状态，`NODE: /robot_state_publisher`订阅这些信息并生成`TOPIC:  /tf`数据；

加上`NODE: /robot_state_publisher`根据`PARAM: urdf`生成的`TOPIC:  /tf_static`，组成机器人本体中的全部tf信息。

