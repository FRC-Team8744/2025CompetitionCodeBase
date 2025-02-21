// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public enum isInAreaEnum {
    NONE(0), // 0
    N6(120), // 300
    N7(180), // 0
    N8(240), // 60
    N9(300), // 120
    N10(0), // 180
    N11(60), // 240
    N17(60), // 240
    N18(0), // 180
    N19(300), // 120
    N20(240), // 60
    N21(180), // 0
    N22(120); // 300

    private double angle;
    private isInAreaEnum(double angle) {
        this.angle = angle;
    }

    public double getAngle() {
        return angle;
    }

    public static isInAreaEnum areaEnum = NONE;
}