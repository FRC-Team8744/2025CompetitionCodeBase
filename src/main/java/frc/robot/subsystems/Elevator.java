// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final TalonFX m_driveMotor;
  private final PositionVoltage position = new PositionVoltage(0);
  public static final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

  // private final Talon
  public Elevator() {
    /*** Motor configuration settings ****/
    elevatorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.StatorCurrentLimit = 10.0;
    elevatorConfig.Slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    elevatorConfig.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    elevatorConfig.Slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    elevatorConfig.Slot0.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    elevatorConfig.Slot0.kI = 0.0; // no output for integrated error
    elevatorConfig.Slot0.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    /*** Configuration settings for velocity and acceleration limiting ****/
    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    elevatorConfig.MotionMagic.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    elevatorConfig.MotionMagic.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    /*** Create motor object ****/
    m_driveMotor = new TalonFX(29);

    /*** Apply configuration to motor ***/
    m_driveMotor.getConfigurator().apply(elevatorConfig);
    m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
    m_driveMotor.setPosition(0);
  }

  public void rotate(double targetPosition) {
    // position.Slot = 0;
    m_driveMotor.setControl(position.withEnableFOC(false).withSlot(0).withPosition(targetPosition));
  }

  public void stopRotate() {
    m_driveMotor.stopMotor();
  }

  public double motorPosition() {
    return m_driveMotor.getPosition().getValueAsDouble();
  }

  public boolean isAtSetpoint() {
    return Math.abs(m_driveMotor.getClosedLoopError().getValueAsDouble()) <= 1.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Kraken Position", m_driveMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Kraken Acceleration", m_driveMotor.getAcceleration().getValueAsDouble());
    SmartDashboard.putNumber("Kraken Error", m_driveMotor.getClosedLoopError().getValueAsDouble());
  }
}
