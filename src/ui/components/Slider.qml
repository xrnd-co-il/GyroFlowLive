// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2021-2022 Adrian <adrian.eddy at gmail>

import QtQuick
import QtQuick.Controls as QQC

QQC.Slider {
    id: slider;
    height: 20 * dpiScale;
    opacity: enabled? 1.0 : 0.5;
    property string unit: "";
    property int precision: 3;
    property real defaultValue: 0;
    Component.onCompleted: defaultValue = value;

    background: Rectangle {
        x: parent.leftPadding
        y: parent.topPadding + parent.availableHeight / 2 - height / 2
        width: parent.availableWidth
        height: 4 * dpiScale;
        radius: 4 * dpiScale;
        color: styleSliderBackground;

        Rectangle {
            width: parent.parent.visualPosition * parent.width
            height: parent.height
            color: styleAccentColor
            radius: parent.radius
        }
    }
    handle: Rectangle {
        x: parent.leftPadding + parent.visualPosition * (parent.availableWidth) - width/2
        y: parent.topPadding + parent.availableHeight / 2 - height / 2
        radius: width;
        height: parent.height * 0.9;
        width: height;
        anchors.verticalCenter: parent.verticalCenter;
        color: styleSliderHandle;
        Rectangle {
            radius: width;
            height: parent.height * 0.7;
            scale: (parent.parent.pressed? 1.1 : parent.parent.hovered? 0.9 : 1.0);
            Ease on scale { duration: 200; }
            width: height;
            anchors.centerIn: parent;
            color: styleAccentColor
        }
    }

    MouseArea {
        anchors.fill: parent;
        property bool isActive: false;
        property real initialValue: 0;
        property bool flickableWasInteractive: true;
        function closestFlickable() {
            let flickable = slider.parent;
            while (flickable && !(flickable instanceof Flickable)) flickable = flickable.parent;
            return flickable;
        }
        onPressed: (mouse) => {
            if (mouse.modifiers & Qt.AltModifier) {
                isActive = true;
                initialValue = slider.value;
                const flick = closestFlickable();
                if (flick) {
                    flickableWasInteractive = flick.interactive;
                    flick.interactive = false;
                }
            } else {
                mouse.accepted = false;
            }
        }
        onMouseXChanged: {
            if (!isActive) return;
            const pos = slider.mapFromItem(this, mouseX, mouseY).x / (slider.width - slider.leftPadding - slider.rightPadding);
            const val = slider.valueAt(pos);
            slider.value = initialValue + (val - initialValue) * 0.2;
        }
        onReleased: {
            if (!isActive) return;
            isActive = false;
            const flick = closestFlickable();
            if (flick) flick.interactive = flickableWasInteractive;
        }
    }

    ToolTip {
        delay: 0;
        parent: handle;
        visible: !isMobile && slider.pressed;
        text: slider.valueAt(slider.position).toFixed(slider.precision) + (slider.unit? " " + slider.unit : "");
        bottomMargin: 5 * dpiScale;
    }
}
