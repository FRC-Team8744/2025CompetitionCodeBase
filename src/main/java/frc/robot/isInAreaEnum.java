// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public enum isInAreaEnum {
    NONE(0),
    N6(300),
    N7(0),
    N8(60),
    N9(120),
    N10(180),
    N11(240),
    N17(240),
    N18(180),
    N19(120),
    N20(60),
    N21(0),
    N22(300);

    private double angle;
    private isInAreaEnum(double angle) {
        this.angle = angle;
    }

    public double getAngle() {
        return angle;
    }

    public static isInAreaEnum areaEnum = NONE;
}