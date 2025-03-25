// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
  public PositionVoltage position = new PositionVoltage(0);
  public VelocityVoltage velocityControl = new VelocityVoltage(0);
  private final Follower followControl;
  public TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
  public TalonFXConfiguration elevatorConfigSlow = new TalonFXConfiguration();
  public Slot0Configs elevatorConfigPIDUp = elevatorConfig.Slot0;
  public Slot1Configs elevatorConfigPIDDown = elevatorConfig.Slot1;
  public boolean elevatorSlot0 = true;
  public double percentOfElevator = 0.9;
  public String scoringPreset = "L4";
  public Elevator() {
    elevatorConfig.Voltage.PeakForwardVoltage = 12;
    elevatorConfig.Voltage.PeakReverseVoltage = -12;
    elevatorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    elevatorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    elevatorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    elevatorConfigPIDUp.kS = 0.0; // Add 0.25 V output to overcome static friction
    elevatorConfigPIDUp.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    elevatorConfigPIDUp.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    elevatorConfigPIDUp.kP = 1.0; // A position error of 2.5 rotations results in 12 V output
    elevatorConfigPIDUp.kI = 0.05; // no output for integrated error
    elevatorConfigPIDDown.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
    elevatorConfigPIDDown.kP = 0.1; // A position error of 2.5 rotations results in 12 V output
    elevatorConfigPIDDown.kI = 0.0; // no output for integrated error
    elevatorConfigPIDDown.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
    elevatorConfig.withSlot0(elevatorConfigPIDUp);
    
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

  // Takes the elevator position from all the way down to the position given
  public void rotate(double targetPosition) {
    // position.Slot = 0;
    position.Slot = 0;
    m_leftElevator.setControl(position.withEnableFOC(false).withPosition(targetPosition));
    m_rightElevator.setControl(followControl);
  }

  // Takes the elevator position from all the way up to all the way down
  public void goDown() {
    position.Slot = 1;
    // m_leftElevator.set(0);
    m_leftElevator.setControl(position.withEnableFOC(false).withPosition(0));
    m_rightElevator.setControl(followControl);
  }

  // Sets the # of rotations for the left motor and makes the right follow suit
  public void rotateVelocity(double velocity) {
    m_leftElevator.setControl(velocityControl.withEnableFOC(false).withVelocity(velocity));
    m_rightElevator.setControl(followControl);
  }

  // Stops both the motors
  public void stopRotate() {
    m_leftElevator.stopMotor();
    m_rightElevator.stopMotor();
  }

  // Returns the position of the left elevator motor
  public double getMotorPosition() {
    return m_leftElevator.getPosition().getValueAsDouble();
  }

  // Returns the absolute value of the distance off from the desired point
  // then if it's between -1 and 1 it will be true otherwise its false
  public boolean isAtSetpoint() {
    return Math.abs(m_leftElevator.getClosedLoopError().getValueAsDouble()) <= 1.0;
  }
  /**
   * This method defines different presets for name and number
   * @param presetNumber Decisions of elevator to go to 0=elevator all the way down, or 1=elevator all the way up
   * @param presetName Give the name of what you decide
   */
  public void setElevatorPreset(double presetNumber, String presetName) {
    percentOfElevator = presetNumber;
    scoringPreset = presetName;
    SmartDashboard.putString("Scoring Preset", presetName);
  }

  // Gives the position of the left elevator in motor rotations and the # the driver sets
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ELevator Position", m_leftElevator.getPosition().getValueAsDouble());
    // SmartDashboard.putNumber("Kraken Acceleration", m_leftElevator.getAcceleration().getValueAsDouble());
    // SmartDashboard.putNumber("Kraken Error", m_leftElevator.getClosedLoopError().getValueAsDouble());
    // SmartDashboard.putBoolean("Is at setpoint", isAtSetpoint());
    // SmartDashboard.putBoolean("Is Slot0?", elevatorSlot0);
    SmartDashboard.putNumber("Elevator Goal Percent", percentOfElevator);
  }
}
