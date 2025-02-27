cmake_minimum_required(VERSION 3.8)
project(rm_decision)
set(THIS_PACKAGE_INCLUDE_DEPENDS
# 如果提示找不到包就调用这个
geometry_msgs
auto_aim_interfaces
    rclcpp
    std_msgs
    behaviortree_cpp
    )
# 自动查找包
find_package (ament_cmake_auto REQUIRED)
# 自动寻找依赖
ament_auto_find_build_dependencies ()
# 添加可执行文件
ament_auto_add_executable(rm_decision
    src/rm_decision.cpp
)
#定义宏函数add_target_dependencies的功能
function(add_target_dependencies target)
target_include_directories(${target} PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
ament_target_dependencies(${target} ${THIS_PACKAGE_INCLUDE_DEPENDS})
endfunction()
# 添加插件
add_library(mynode_plugin SHARED bt_node_self/mynode.cpp)
target_compile_definitions(mynode_plugin PRIVATE  BT_PLUGIN_EXPORT )
add_target_dependencies(mynode_plugin)
# 自动打包
ament_auto_package ()