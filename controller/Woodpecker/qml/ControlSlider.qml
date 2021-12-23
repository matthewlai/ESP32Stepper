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

Text {
    id: sliderRoot
    width: parent.width
    height: GameSettings.fieldHeight
    horizontalAlignment: Text.AlignHCenter
    verticalAlignment: Text.AlignBottom
    color: GameSettings.textColor
    font.pixelSize: GameSettings.mediumFontSize

    property bool is_range: true

    Loader {
        sourceComponent: is_range ? rangeSliderComponent : sliderComponent

        width: sourceComponent.width
        height: sourceComponent.height
        anchors.top: sliderRoot.bottom
        anchors.topMargin: sliderRoot.width * 0.03
        anchors.horizontalCenter: sliderRoot.horizontalCenter

        Component {
            id: rangeSliderComponent

            RangeSlider {
                id: rangeSlider
                width: sliderRoot.width * 0.8
                from: 0.0
                first.value: 0.25
                second.value: 0.75
                to: 1.0

                first.handle: Rectangle {
                    x: rangeSlider.leftPadding + rangeSlider.first.visualPosition * (rangeSlider.availableWidth - width)
                    y: rangeSlider.topPadding + rangeSlider.availableHeight / 2 - height / 2
                    implicitWidth: 35
                    implicitHeight: 35
                    radius: 10
                    color: rangeSlider.first.pressed ? GameSettings.sliderColor : GameSettings.buttonPressedColor
                }

                second.handle: Rectangle {
                    x: rangeSlider.leftPadding + rangeSlider.second.visualPosition * (rangeSlider.availableWidth - width)
                    y: rangeSlider.topPadding + rangeSlider.availableHeight / 2 - height / 2
                    implicitWidth: 35
                    implicitHeight: 35
                    radius: 10
                    color: rangeSlider.second.pressed ? GameSettings.sliderColor : GameSettings.buttonPressedColor
                }

                background: Rectangle {
                    x: rangeSlider.leftPadding
                    y: rangeSlider.topPadding + rangeSlider.availableHeight / 2 - height / 2
                    implicitWidth: 200
                    implicitHeight: 8
                    width: rangeSlider.availableWidth
                    height: implicitHeight * 0.7
                    radius: 2
                    color: GameSettings.backgroundColor

                    Rectangle {
                        x: rangeSlider.first.visualPosition * parent.width
                        width: rangeSlider.second.visualPosition * parent.width - x
                        height: parent.height
                        color: "white"
                        radius: 2
                    }
                }

                // Range dragging rectange, adapted from: https://stackoverflow.com/questions/53718834/dragging-the-range-of-a-rangeslider
                MouseArea {
                    anchors.left: rangeSlider.first.handle.right
                    anchors.right: rangeSlider.second.handle.left
                    height: rangeSlider.first.handle.height
                    property real dx: 0
                    onPressed: dx = mouseX
                    onPositionChanged: {
                        var d = ((mouseX - dx) / rangeSlider.width) * Math.abs(rangeSlider.to - rangeSlider.from)
                        if ((d + rangeSlider.first.value) < rangeSlider.from) d = rangeSlider.from - rangeSlider.first.value
                        if ((d + rangeSlider.second.value) > rangeSlider.to) d = rangeSlider.to - rangeSlider.second.value
                        rangeSlider.first.value += d
                        rangeSlider.second.value += d
                        dx = mouseX
                    }
                }
            }
        }

        Component {
            id: sliderComponent

            Slider {
                id: slider
                width: sliderRoot.width * 0.8
                from: 0.0
                value: 0.0
                to: 1.0

                handle: Rectangle {
                    x: slider.leftPadding + slider.visualPosition * (slider.availableWidth - width)
                    y: slider.topPadding + slider.availableHeight / 2 - height / 2
                    implicitWidth: 35
                    implicitHeight: 35
                    radius: 10
                    color: slider.pressed ? GameSettings.sliderColor : GameSettings.buttonPressedColor
                }

                background: Rectangle {
                    x: slider.leftPadding
                    y: slider.topPadding + slider.availableHeight / 2 - height / 2
                    implicitWidth: 200
                    implicitHeight: 8
                    width: slider.availableWidth
                    height: implicitHeight * 0.7
                    radius: 2
                    color: GameSettings.backgroundColor

                    Rectangle {
                        x: 0
                        width: slider.visualPosition * parent.width
                        height: parent.height
                        color: "white"
                        radius: 2
                    }
                }
            }
        }
    }
}
