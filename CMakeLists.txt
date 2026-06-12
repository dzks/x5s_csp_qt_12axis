cmake_minimum_required(VERSION 3.10)
project(x5s_igh_csp_qt_direct LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include_directories(include)
include_directories(/opt/etherlab/include)
include_directories(/usr/local/include)

link_directories(
    /opt/etherlab/lib
    /usr/local/lib
)

set(COMMON_SOURCES
    src/pdo_config.cpp
    src/cia402.cpp
    src/x5s_axis.cpp
    src/ethercat_master.cpp
    src/time_utils.cpp
)

# Console fallback target. Useful for testing direct-CSP logic without Qt UI.
add_executable(x5s_csp_console_direct
    src/main_console_direct.cpp
    src/control_worker.cpp
    ${COMMON_SOURCES}
)

target_link_libraries(x5s_csp_console_direct
    ethercat
    pthread
)

# Qt UI target. Supports Qt6 first, then Qt5.
find_package(Qt6 COMPONENTS Widgets QUIET)
if(Qt6_FOUND)
    add_executable(x5s_csp_qt
        src/main_qt.cpp
        src/control_worker.cpp
        ${COMMON_SOURCES}
    )
    target_link_libraries(x5s_csp_qt Qt6::Widgets ethercat pthread)
else()
    find_package(Qt5 COMPONENTS Widgets QUIET)
    if(Qt5_FOUND)
        add_executable(x5s_csp_qt
            src/main_qt.cpp
            src/control_worker.cpp
            ${COMMON_SOURCES}
        )
        target_link_libraries(x5s_csp_qt Qt5::Widgets ethercat pthread)
    else()
        message(WARNING "Qt5/Qt6 Widgets development files were not found. x5s_csp_qt will not be built. Install qtbase5-dev or Qt6 Widgets dev package.")
    endif()
endif()
