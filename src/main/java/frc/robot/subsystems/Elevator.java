// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final TalonFX m_driveMotor;
  private final PositionVoltage position = new PositionVoltage(0);
  public static final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
  public static final Slot0Configs elevatorConfigPID = elevatorConfig.Slot0;
  public double targetPosition = 25;
  // private final Talon
  public Elevator() {
    elevatorConfig.Voltage.PeakForwardVoltage = 12;
    elevatorConfig.Voltage.PeakReverseVoltage = -12;
    elevatorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    elevatorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    elevatorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    // elevatorConfigPID.kV = 0.0;
    // elevatorConfigPID.kP = 1.0;
    // elevatorConfigPID.kI = 0.0;
    // elevatorConfigPID.kD = 0.0;
    elevatorConfigPID.kS = 1.0; // Add 0.25 V output to overcome static friction
    elevatorConfigPID.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    elevatorConfigPID.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    elevatorConfigPID.kP = 24.0; // A position error of 2.5 rotations results in 12 V output
    elevatorConfigPID.kI = 0.0; // no output for integrated error
    elevatorConfigPID.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
    elevatorConfig.withSlot0(elevatorConfigPID);
    // elevatorMotionMagicConfig.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    // elevatorMotionMagicConfig.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    // elevatorMotionMagicConfig.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
    m_driveMotor = new TalonFX(29);

    m_driveMotor.getConfigurator().apply(elevatorConfig);
    // m_driveMotor.getConfigurator().apply(elevatorConfigPID);
    m_driveMotor.getConfigurator().setPosition(0);
    m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
    m_driveMotor.setPosition(0);
  }

  public void rotate(double feedForward) {
    position.Slot = 0;
    m_driveMotor.setControl(position.withSlot(0).withPosition(targetPosition).withFeedForward(feedForward));
    // m_driveMotor.setPosition(10);
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
