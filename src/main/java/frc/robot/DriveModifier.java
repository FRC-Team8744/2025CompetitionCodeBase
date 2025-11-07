// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public abstract class DriveModifier {
    private boolean firstRun = true;
    public final boolean actingOnX;
    public final boolean actingOnY;
    public final boolean actingOnRot;
    protected DriveModifier(boolean x, boolean y, boolean rot) {
        actingOnX = x;
        actingOnY = y;
        actingOnRot = rot;
    }
    protected void initialize(){}
    public abstract boolean shouldRun(DriveSubsystem drive);
    protected abstract void doExecute(DriveSubsystem drive);
    protected void cleanUp(){}
    public final void execute(DriveSubsystem drive){
        if(shouldRun(drive)) {
            if (firstRun) {
                initialize();
                firstRun = false;
            }
            doExecute(drive);
        } else {
            cleanUp();
            firstRun = true;
        }
    }
}