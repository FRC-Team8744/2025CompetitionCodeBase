// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFX m_indexMotorL;
  private TalonFX m_indexMotorR;
  private TalonFX m_intakeMotor;
  private TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
  private Slot0Configs intakeConfigPID = intakeConfig.Slot0;
  public Intake() {
    intakeConfig.Voltage.PeakForwardVoltage = 12;
    intakeConfig.Voltage.PeakReverseVoltage = -12;
    intakeConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    intakeConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    intakeConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    intakeConfigPID.kS = 0.0; // Add 0.25 V output to overcome static friction
    intakeConfigPID.kV = 1.0; // A velocity target of 1 rps results in 0.12 V output
    intakeConfigPID.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    intakeConfigPID.kP = 1.0; // A position error of 2.5 rotations results in 12 V output
    intakeConfigPID.kI = 0.05; // no output for integrated error
    intakeConfigPID.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
    intakeConfig.withSlot0(intakeConfigPID);
    
    m_indexMotorL = new TalonFX(Constants.SwerveConstants.kIndexerMotorPortL);
    m_indexMotorR = new TalonFX(Constants.SwerveConstants.kIndexerMotorPortR);
    m_intakeMotor = new TalonFX(Constants.SwerveConstants.kIntakeRollerMotorPort);

    m_intakeMotor.getConfigurator().apply(intakeConfig);
    m_intakeMotor.setNeutralMode(NeutralModeValue.Coast);
    m_intakeMotor.setPosition(0);
    m_intakeMotor.getPosition();

    m_indexMotorL.getConfigurator().apply(intakeConfig);
    m_indexMotorL.setNeutralMode(NeutralModeValue.Coast);
    m_indexMotorL.setPosition(0);

    m_indexMotorR.getConfigurator().apply(intakeConfig);
    m_indexMotorR.setNeutralMode(NeutralModeValue.Coast);
    m_indexMotorR.setPosition(0);
  }

  // Runs the motors at set speeds for the indexer
  public void runIndexer(double speedL, double speedR) {
    m_indexMotorL.set(speedL);
    m_indexMotorR.set(speedR);
  }

  // Runs the motor for teh intake
  public void runIntake(double speed) {
    m_intakeMotor.set(speed);
  }

  // Sets the intake motor to 1 and the intake motors for what the driver sets
  public void runIntakeAndIndexer(double speedL, double speedR) {
    m_intakeMotor.set(1);
    m_indexMotorL.set(speedL);
    m_indexMotorR.set(speedR);
  }

  // Stops the indexer motors
  public void stopIndexer() {
    m_indexMotorL.stopMotor();
    m_indexMotorR.stopMotor();
  }

  // Stops the intake motor
  public void stopIntake() {
    m_intakeMotor.stopMotor();
  }

  // Stops both the intake and index motors
  public void stopBoth() {
    m_intakeMotor.stopMotor();
    m_indexMotorL.stopMotor();
    m_indexMotorR.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
