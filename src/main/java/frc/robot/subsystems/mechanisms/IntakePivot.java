// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePivot extends SubsystemBase {
  /** Creates a new IntakePivot. */
  private final TalonFX m_intakePivot;
  private final double startingPositionRotations = 0;
  private final double minimumAngle = 0;
  private final double maximumAngle = 110;
  public double goalAngle;
  private final PositionVoltage goalPosition = new PositionVoltage(startingPositionRotations);
  public IntakePivot() {
    m_intakePivot = new TalonFX(31);

    m_intakePivot.getConfigurator().apply(Constants.intakePivotConfig);
    m_intakePivot.setNeutralMode(NeutralModeValue.Brake);
    m_intakePivot.setPosition(startingPositionRotations);
  }

  public void intakeDown(double angle) {
    if (angle < minimumAngle) {angle = minimumAngle;}
    if (angle > maximumAngle) {angle = maximumAngle;}
    m_intakePivot.setControl(goalPosition.withEnableFOC(false).withSlot(0).withPosition(angle / 360));
  }

  public double getPositionAngle() {
    return m_intakePivot.getPosition().getValueAsDouble() * 360;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
