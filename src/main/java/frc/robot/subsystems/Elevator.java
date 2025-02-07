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
  // public static final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
  // public static final Slot0Configs elevatorConfigPID = elevatorConfig.Slot0;
  public double targetPosition = 25;
  // private final Talon
  public Elevator() {
    m_driveMotor = new TalonFX(29);

    m_driveMotor.getConfigurator().apply(Constants.elevatorConfig);
    m_driveMotor.getConfigurator().setPosition(0);
    m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
    m_driveMotor.setPosition(0);
  }

  public void rotate() {
    position.Slot = 0;
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
