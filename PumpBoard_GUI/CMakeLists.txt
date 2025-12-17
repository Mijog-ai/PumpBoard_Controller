cmake_minimum_required(VERSION 3.16)
project(PumpBoardGUI VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Find Qt packages
find_package(Qt6 COMPONENTS Core Gui Widgets SerialPort Charts QUIET)
if(NOT Qt6_FOUND)
    find_package(Qt5 5.15 COMPONENTS Core Gui Widgets SerialPort Charts REQUIRED)
    set(QT_VERSION_MAJOR 5)
else()
    set(QT_VERSION_MAJOR 6)
endif()

# Source files
set(SOURCES
    src/main.cpp
    src/mainwindow.cpp
    src/serialcommunication.cpp
    src/dashboardwidget.cpp
    src/controlpanelwidget.cpp
    src/pidtuningwidget.cpp
    src/datalogger.cpp
    src/realtimechartwidget.cpp
    src/gaugewidget.cpp
)

# Header files
set(HEADERS
    include/pumpboard_protocol.h
    include/mainwindow.h
    include/serialcommunication.h
    include/dashboardwidget.h
    include/controlpanelwidget.h
    include/pidtuningwidget.h
    include/datalogger.h
    include/realtimechartwidget.h
    include/gaugewidget.h
)

# UI files
set(UI_FILES
    ui/mainwindow.ui
)

# Resource files
set(RESOURCE_FILES
    resources/resources.qrc
)

# Create executable
add_executable(${PROJECT_NAME}
    ${SOURCES}
    ${HEADERS}
    ${UI_FILES}
    ${RESOURCE_FILES}
)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Link Qt libraries
if(QT_VERSION_MAJOR EQUAL 6)
    target_link_libraries(${PROJECT_NAME} PRIVATE
        Qt6::Core
        Qt6::Gui
        Qt6::Widgets
        Qt6::SerialPort
        Qt6::Charts
    )
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        Qt5::SerialPort
        Qt5::Charts
    )
endif()

# Platform-specific settings
if(WIN32)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        WIN32_EXECUTABLE TRUE
    )
endif()

# Install target
install(TARGETS ${PROJECT_NAME}
    BUNDLE DESTINATION .
    RUNTIME DESTINATION bin
)
