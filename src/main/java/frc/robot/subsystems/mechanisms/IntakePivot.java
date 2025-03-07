// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePivot extends SubsystemBase {
  /** Creates a new IntakePivot. */
  private final TalonFX m_intakePivot;
  private final TalonFXConfiguration intakePivotConfig = new TalonFXConfiguration();
  private final Slot0Configs intakePivotConfigPID = intakePivotConfig.Slot0;
  private final double startingPositionRotations = 0;
  private final double minimumAngle = -4000;
  private final double maximumAngle = 0;
  private final PositionVoltage goalPosition = new PositionVoltage(startingPositionRotations);
  public IntakePivot() {
    intakePivotConfig.Voltage.PeakForwardVoltage = 12;
    intakePivotConfig.Voltage.PeakReverseVoltage = -12;
    intakePivotConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    intakePivotConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    intakePivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    intakePivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakePivotConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    intakePivotConfigPID.kS = 24.0; // Add 0.25 V output to overcome static friction
    intakePivotConfigPID.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    intakePivotConfigPID.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    intakePivotConfigPID.kP = 1.0; // A position error of 2.5 rotations results in 12 V output
    intakePivotConfigPID.kI = 0.0; // no output for integrated error
    intakePivotConfigPID.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
    intakePivotConfig.withSlot0(intakePivotConfigPID);

    m_intakePivot = new TalonFX(Constants.SwerveConstants.kIntakePivotMotorPort);

    m_intakePivot.getConfigurator().apply(intakePivotConfig);
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
    SmartDashboard.putNumber("Intake Pivot Motor Angle", m_intakePivot.getPosition().getValueAsDouble() * 360);
  }
}
