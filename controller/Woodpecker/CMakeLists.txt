cmake_minimum_required(VERSION 3.16)
project(woodpecker LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 COMPONENTS Core)
find_package(Qt6 COMPONENTS Gui)
find_package(Qt6 COMPONENTS Qml)
find_package(Qt6 COMPONENTS Quick)
find_package(Qt6 COMPONENTS Bluetooth)

qt_add_executable(woodpecker
    bluetoothbaseclass.cpp bluetoothbaseclass.h
    connectionhandler.cpp connectionhandler.h
    devicefinder.cpp devicefinder.h
    devicehandler.cpp devicehandler.h
    deviceinfo.cpp deviceinfo.h
    woodpecker-global.h
    main.cpp
)
set_target_properties(woodpecker PROPERTIES
    WIN32_EXECUTABLE TRUE
    MACOSX_BUNDLE TRUE
)
target_link_libraries(woodpecker PUBLIC
    Qt::Bluetooth
    Qt::Core
    Qt::Gui
    Qt::Qml
    Qt::Quick
)

if (APPLE)
    if (IOS)
        set_target_properties(woodpecker PROPERTIES
            MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/Info.plist"
        )
    else()
        # Using absolute path for shared plist files is a Ninja bug workaround
        get_filename_component(SHARED_PLIST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../shared ABSOLUTE)
        set_target_properties(woodpecker PROPERTIES
            MACOSX_BUNDLE_INFO_PLIST "${SHARED_PLIST_DIR}/Info.cmake.macos.plist"
        )
    endif()
endif()

# Resources:
set(qml_resource_files
    "qml/App.qml"
    "qml/BluetoothAlarmDialog.qml"
    "qml/BottomLine.qml"
    "qml/Connect.qml"
    "qml/GameButton.qml"
    "qml/GamePage.qml"
    "qml/GameSettings.qml"
    "qml/Measure.qml"
    "qml/SplashScreen.qml"
    "qml/Stats.qml"
    "qml/StatsLabel.qml"
    "qml/TitleBar.qml"
    "qml/main.qml"
    "qml/qmldir"
)

qt6_add_resources(woodpecker "qml"
    PREFIX
        "/"
    FILES
        ${qml_resource_files}
)
set(images_resource_files
    "qml/images/bt_off_to_on.png"
    "qml/images/heart.png"
    "qml/images/logo.png"
)

qt6_add_resources(woodpecker "images"
    PREFIX
        "/"
    FILES
        ${images_resource_files}
)
