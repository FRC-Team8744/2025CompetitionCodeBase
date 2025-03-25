// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public enum isInAreaEnum {
    NONE(0, 0), // 0
    N6(120, 6), // 300
    N7(180, 7), // 0
    N8(240, 8), // 60
    N9(300, 9), // 120
    N10(0, 10), // 180
    N11(60, 11), // 240
    N17(60, 17), // 240
    N18(0, 18), // 180
    N19(300, 19), // 120
    N20(240, 20), // 60
    N21(180, 21), // 0
    N22(120, 22); // 300

    private double angle;
    private int aprilTag;
    private isInAreaEnum(double angle, int aprilTag) {
        this.angle = angle;
        this.aprilTag = aprilTag;
    }

    public double getAngle() {
        return angle;
    }

    public int getAprilTag() {
        return aprilTag;
    }

    public static isInAreaEnum areaEnum = NONE;
}