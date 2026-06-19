cmake_minimum_required(VERSION 3.10)
project(x5s_igh_csp_qt_direct LANGUAGES CXX)

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
)

# Qt UI target. Supports Qt6 first, then Qt5.
find_package(Qt6 COMPONENTS Widgets QUIET)
if(Qt6_FOUND)
    add_executable(x5s_csp_qt
        src/Main/main_qt.cpp
        src/control_worker.cpp
        ${COMMON_SOURCES}
    )
    target_link_libraries(x5s_csp_qt Qt6::Widgets ethercat pthread)
else()
    find_package(Qt5 COMPONENTS Widgets QUIET)
    if(Qt5_FOUND)
        add_executable(x5s_csp_qt
            src/Main/main_qt.cpp
            src/control_worker.cpp
            ${COMMON_SOURCES}
        )
        target_link_libraries(x5s_csp_qt Qt5::Widgets ethercat pthread)
    else()
        message(WARNING "Qt5/Qt6 Widgets development files were not found. x5s_csp_qt will not be built. Install qtbase5-dev or Qt6 Widgets dev package.")
    endif()
endif()

add_executable(main_InitialPosition
    src/Main/main_InitialPosition.cpp
    ${KINEMATICS_SOURCES}
    ${MOTOR_SOURCES}
    ${COMMON_SOURCES}
)
target_link_libraries(main_InitialPosition ethercat pthread)

add_executable(test
    src/Main/test.cpp
    ${KINEMATICS_SOURCES}
    ${COMMON_SOURCES}
    ${MOTOR_SOURCES}
)
target_link_libraries(test ethercat pthread)