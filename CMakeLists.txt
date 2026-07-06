cmake_minimum_required(VERSION 3.10)
project(x5s_igh_csp_qt_direct LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Project headers are included from the include/ root, e.g.
#   #include "Ethercat/ethercat_master.hpp"
#   #include "QT/shared_state.hpp"
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    /opt/etherlab/include
    /usr/local/include
)

link_directories(
    /opt/etherlab/lib
    /usr/local/lib
)

# 通信相关代码
set(COMMON_SOURCES
    src/Ethercat/pdo_config.cpp
    src/Ethercat/cia402.cpp
    src/Ethercat/x5s_axis.cpp
    src/Ethercat/ethercat_master.cpp
    src/time_utils.cpp
)

# 运动学相关代码
set(KINEMATICS_SOURCES
    src/Kinematics/parallel_kinematics.cpp
    src/Kinematics/axis_unit_converter.cpp
)

# 电机相关代码
set(MOTOR_SOURCES
    src/Motor/Motor_Control.cpp
    src/Motor/Parallel_Control.cpp
    src/Motor/Multi_Parallel_Control.cpp
)

# 轨迹规划代码
set(TRAJECTORY
    src/Trajectory/count_trajectory_generator.cpp
)

set(QT
    src/QT/PlateShapeInitWindow.cpp
    src/QT/PlateShapeView.cpp
    src/QT/PlateShapeMotionWorker.cpp
    include/QT/PlateShapeTypes.hpp
    include/QT/PlateShapeInitWindow.hpp
    include/QT/PlateShapeView.hpp
    include/QT/PlateShapeMotionWorker.hpp
)

# Qt UI target. Supports Qt6 first, then Qt5.
find_package(Qt6 COMPONENTS Widgets QUIET)
if(Qt6_FOUND)
    add_executable(x5s_csp_qt
        src/QT/main_qt.cpp
        src/QT/control_worker.cpp
        ${COMMON_SOURCES}
    )
    target_link_libraries(x5s_csp_qt Qt6::Widgets ethercat pthread)
else()
    find_package(Qt5 COMPONENTS Widgets QUIET)
    if(Qt5_FOUND)
        add_executable(x5s_csp_qt
            src/QT/main_qt.cpp
            src/QT/control_worker.cpp
            ${COMMON_SOURCES}
        )
        target_link_libraries(x5s_csp_qt Qt5::Widgets ethercat pthread)
    else()
        message(WARNING "Qt5/Qt6 Widgets development files were not found. x5s_csp_qt will not be built. Install qtbase5-dev or Qt6 Widgets dev package.")
    endif()
endif()

# 测试主程序
add_executable(test
    src/Main/test.cpp
    ${KINEMATICS_SOURCES}
    ${COMMON_SOURCES}
    ${MOTOR_SOURCES}
    ${TRAJECTORY}
)
target_link_libraries(test ethercat pthread)

# 初始板型构建QT程序
add_executable(main_plateshapeInit_qt
    src/QT/main_plateshapeInit_qt.cpp
    ${QT}
    ${COMMON_SOURCES}
    ${MOTOR_SOURCES}
    ${KINEMATICS_SOURCES}
    ${TRAJECTORY}
)

target_include_directories(main_plateshapeInit_qt PRIVATE include)
target_link_libraries(main_plateshapeInit_qt Qt5::Widgets ethercat pthread)


# 测试初始板型主程序
add_executable(InitialPosition
    src/Main/InitialPosition.cpp
    ${KINEMATICS_SOURCES}
    ${COMMON_SOURCES}
    ${MOTOR_SOURCES}
    ${TRAJECTORY}
)
target_link_libraries(InitialPosition ethercat pthread)