// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final TalonFX m_leftElevator;
  private final TalonFX m_rightElevator;
  private final PositionVoltage position = new PositionVoltage(0);
  private final Follower followControl;
  public Elevator() {
    m_leftElevator = new TalonFX(29);
    m_rightElevator = new TalonFX(30);

    followControl = new Follower(m_leftElevator.getDeviceID(), true);

    m_rightElevator.getConfigurator().apply(Constants.elevatorConfig);
    m_rightElevator.setNeutralMode(NeutralModeValue.Brake);
    m_rightElevator.setPosition(0);

    m_leftElevator.getConfigurator().apply(Constants.elevatorConfig);
    m_leftElevator.setNeutralMode(NeutralModeValue.Brake);
    m_leftElevator.setPosition(0);

    m_leftElevator.setSafetyEnabled(true);
  }

  public void rotate(double targetPosition) {
    position.Slot = 0;
    m_leftElevator.setControl(position.withEnableFOC(false).withSlot(0).withPosition(targetPosition));
    m_rightElevator.setControl(followControl);
  }

  public void stopRotate() {
    m_leftElevator.stopMotor();
    m_rightElevator.stopMotor();
  }

  public double motorPosition() {
    return m_leftElevator.getPosition().getValueAsDouble();
  }

  public boolean isAtSetpoint() {
    return Math.abs(m_leftElevator.getClosedLoopError().getValueAsDouble()) <= 1.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Kraken Position", m_leftElevator.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Kraken Acceleration", m_leftElevator.getAcceleration().getValueAsDouble());
    SmartDashboard.putNumber("Kraken Error", m_leftElevator.getClosedLoopError().getValueAsDouble());
  }
}
