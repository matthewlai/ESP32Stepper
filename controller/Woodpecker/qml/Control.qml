/***************************************************************************
**
** Copyright (C) 2017 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the examples of the QtBluetooth module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

import QtQuick 6.0
import QtQuick.Controls 6.0

GamePage {
    id: controlPage

    errorMessage: deviceHandler.error
    infoMessage: deviceHandler.info

    property bool started: false

    function close()
    {
        deviceHandler.disconnectService();
        app.prevPage();
    }

    function start()
    {
        started = true
        deviceHandler.start();
        startStopButtonText.text = qsTr("STOP")
        startStopButton.baseColor = GameSettings.stopButtonColor
        startStopButton.checkColor()
    }

    function stop()
    {
        started = false
        deviceHandler.stop();
        startStopButtonText.text = qsTr("START")
        startStopButton.baseColor = GameSettings.startButtonColor
        startStopButton.checkColor()
    }

    Rectangle {
        anchors.top: parent.top
        anchors.bottom: startStopButton.top
        anchors.topMargin: GameSettings.fieldMargin + messageHeight
        anchors.bottomMargin: GameSettings.fieldMargin
        anchors.horizontalCenter: parent.horizontalCenter
        width: parent.width - GameSettings.fieldMargin*2
        color: GameSettings.viewColor

        ControlSlider {
            id: rangeSlider
            anchors.top: parent.top
            is_range: true
            text: qsTr("Range")
        }

        ControlSlider {
            id: speedSlider
            anchors.top: rangeSlider.bottom
            anchors.topMargin: GameSettings.fieldMargin
            is_range: false
            text: qsTr("Speed")
        }

        Text {
            id: patternsText
            anchors.top: speedSlider.bottom
            anchors.topMargin: GameSettings.fieldMargin
            width: parent.width
            height: GameSettings.fieldHeight
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignBottom
            color: GameSettings.textColor
            font.pixelSize: GameSettings.mediumFontSize
            text: qsTr("Pattern")
        }

        GridView {
            id: patterns
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: patternsText.bottom
            anchors.topMargin: parent.width * 0.03
            width: parent.width * 0.8
            height: cellHeight * 2
            cellWidth: width / 4
            cellHeight: cellWidth

            property var modes: ["sine", "triangle", "square", "sawtooth"]
            property int selectedIndex: 0

            model: modes
            delegate: Item {
                width: patterns.cellWidth
                height: patterns.cellHeight

                Rectangle {
                    id: rect
                    anchors.centerIn: parent
                    color: index == patterns.selectedIndex ? GameSettings.buttonPressedColor : GameSettings.backgroundColor
                    width: patterns.cellWidth * 0.85
                    height: width
                }

                Image {
                    source: "images/patterns/" + patterns.modes[index] + ".png"
                    anchors.fill: rect
                }

                MouseArea {
                    id: mouseArea
                    anchors.fill: rect
                    onClicked: {
                        patterns.selectedIndex = index
                    }
                }
            }
        }
/*
            delegate: Rectangle {
                id: box
                height:GameSettings.fieldHeight * 1.2
                width: devices.width
                color: baseColor

                property color baseColor: index % 2 === 0 ? GameSettings.delegate1Color : GameSettings.delegate2Color
                property color pressedColor: GameSettings.buttonPressedColor

                MouseArea {
                    id: mouseArea
                    anchors.fill: parent
                    onPressed: box.color = pressedColor
                    onReleased: box.color = baseColor
                    onClicked: {
                        deviceFinder.connectToService(modelData.deviceAddress);
                        app.showPage(app.controlPageIndex)
                    }
                }

                Text {
                    id: device
                    font.pixelSize: GameSettings.smallFontSize
                    text: modelData.deviceName
                    anchors.top: parent.top
                    anchors.topMargin: parent.height * 0.1
                    anchors.leftMargin: parent.height * 0.1
                    anchors.left: parent.left
                    color: GameSettings.textColor
                }

                Text {
                    id: deviceAddress
                    font.pixelSize: GameSettings.smallFontSize
                    text: modelData.deviceAddress
                    anchors.bottom: parent.bottom
                    anchors.bottomMargin: parent.height * 0.1
                    anchors.rightMargin: parent.height * 0.1
                    anchors.right: parent.right
                    color: Qt.darker(GameSettings.textColor)
                }
            }
            */
    }

    GameButton {
        id: startStopButton
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.bottom
        anchors.bottomMargin: GameSettings.fieldMargin
        width: parent.width - GameSettings.fieldMargin*2
        height: GameSettings.fieldHeight
        enabled: deviceHandler.alive
        radius: GameSettings.buttonRadius
        baseColor: GameSettings.startButtonColor

        onClicked: started ? stop() : start()

        Text {
            id: startStopButtonText
            anchors.centerIn: parent
            font.pixelSize: GameSettings.smallFontSize
            text: qsTr("START")
            color: startStopButton.enabled ? GameSettings.textColor : GameSettings.disabledTextColor
        }
    }
}
