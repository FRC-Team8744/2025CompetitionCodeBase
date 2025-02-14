// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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
  public TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
  private final Slot0Configs elevatorConfigPID = elevatorConfig.Slot0;
  public Elevator() {
    elevatorConfig.Voltage.PeakForwardVoltage = 12;
    elevatorConfig.Voltage.PeakReverseVoltage = -12;
    elevatorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    elevatorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    elevatorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    elevatorConfigPID.kS = 0.0; // Add 0.25 V output to overcome static friction
    elevatorConfigPID.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    elevatorConfigPID.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    elevatorConfigPID.kP = 7.0; // A position error of 2.5 rotations results in 12 V output
    elevatorConfigPID.kI = 0.05; // no output for integrated error
    elevatorConfigPID.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
    elevatorConfig.withSlot0(elevatorConfigPID);

    m_leftElevator = new TalonFX(Constants.SwerveConstants.kLeftElevatorMotorPort);
    m_rightElevator = new TalonFX(Constants.SwerveConstants.kRightElevatorMotorPort);

    followControl = new Follower(m_leftElevator.getDeviceID(), true);

    m_rightElevator.getConfigurator().apply(elevatorConfig);
    m_rightElevator.setNeutralMode(NeutralModeValue.Brake);
    m_rightElevator.setPosition(0);

    m_leftElevator.getConfigurator().apply(elevatorConfig);
    m_leftElevator.setNeutralMode(NeutralModeValue.Brake);
    m_leftElevator.setPosition(0);
  }

  public void rotate(double targetPosition) {
    // position.Slot = 0;
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
    SmartDashboard.putBoolean("Is at setpoint", isAtSetpoint());
  }
}
