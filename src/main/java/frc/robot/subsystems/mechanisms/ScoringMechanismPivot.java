// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ScoringMechanismPivot extends SubsystemBase {
  /** Creates a new ScoringMechanismPivot. */
  private final TalonFX m_scoringMechPivot;
  private final TalonFXConfiguration scoringMechPivotConfig = new TalonFXConfiguration();
  private final Slot0Configs scoringMechPivotConfigPID = scoringMechPivotConfig.Slot0;
  private final Slot1Configs scoringMechPivotConfigResetPID = scoringMechPivotConfig.Slot1;
  private final double startingPositionRotations = 0;
  private final double minimumAngle = -260;
  private final double maximumAngle = 0;
  private final PositionVoltage goalPosition = new PositionVoltage(startingPositionRotations);
  private final VelocityVoltage velocity = new VelocityVoltage(0);
  public ScoringMechanismPivot() {
    // Scoring Mechanism Pivot Configs
    scoringMechPivotConfig.Voltage.PeakForwardVoltage = 12;
    scoringMechPivotConfig.Voltage.PeakReverseVoltage = -12;
    scoringMechPivotConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    scoringMechPivotConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    scoringMechPivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    scoringMechPivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    scoringMechPivotConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    scoringMechPivotConfigPID.kS = 0.5; // Add 0.25 V output to overcome static friction
    scoringMechPivotConfigPID.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    scoringMechPivotConfigPID.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    scoringMechPivotConfigPID.kP = 3.5; // A position error of 2.5 rotations results in 12 V output
    scoringMechPivotConfigPID.kI = 0.0; // no output for integrated error
    scoringMechPivotConfigPID.kD = 0.05; // A velocity error of 1 rps results in 0.1 V output
    scoringMechPivotConfigResetPID.kP = 0.0;
    scoringMechPivotConfigResetPID.kI = 0.0;
    scoringMechPivotConfigResetPID.kD = 0.0;
    scoringMechPivotConfig.withSlot0(scoringMechPivotConfigPID);
    scoringMechPivotConfig.withSlot1(scoringMechPivotConfigResetPID);

    m_scoringMechPivot = new TalonFX(Constants.SwerveConstants.kScoringMechanismPivotMotorPort);

    m_scoringMechPivot.getConfigurator().apply(scoringMechPivotConfig);
    m_scoringMechPivot.setNeutralMode(NeutralModeValue.Brake);
    m_scoringMechPivot.setPosition(startingPositionRotations);
  }

  public void rotatePivot(double angle) {
    if (angle < minimumAngle) {angle = minimumAngle;}
    if (angle > maximumAngle) {angle = maximumAngle;}
    m_scoringMechPivot.setControl(goalPosition.withEnableFOC(false).withSlot(0).withPosition(angle / 360));
  }

  public void goDown() {
    m_scoringMechPivot.setControl(velocity.withEnableFOC(false).withVelocity(0).withFeedForward(1).withSlot(1));
  }

  public void stall() {
    m_scoringMechPivot.setControl(velocity.withEnableFOC(false).withVelocity(0).withFeedForward(-1).withSlot(1));
  }

  public void resetEncoder() {
    m_scoringMechPivot.stopMotor();
    m_scoringMechPivot.setPosition(startingPositionRotations);
  }

  public double getPositionAngle() {
    return m_scoringMechPivot.getPosition().getValueAsDouble() * 360;
  }

  public boolean isAtSetpoint() {
    return Math.abs(m_scoringMechPivot.getClosedLoopError().getValueAsDouble()) <= 1/18;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Scoring Mech Pivot Angle", getPositionAngle());
    // SmartDashboard.putBoolean("Is scoring mech pivot at setpoint", isAtSetpoint());
  }
}
