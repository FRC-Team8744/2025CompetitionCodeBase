// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScoringMechSensor extends SubsystemBase {
  /** Creates a new ScoringMechSensor. */
  private DigitalInput inputIR = new DigitalInput(0);
  public ScoringMechSensor() {}

  public boolean getScoringSensor() {
    return inputIR.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Scoring Mech Sensor", inputIR.get());
  }
}
