// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private static final boolean TUNE_PID_MODE = true;  // Only set to true when tuning PID parameters

  /* Manual tuning typically follows this process:
  https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/closed-loop-requests.html
  1. Set all gains to zero.
  2. Determine Kg if using an elevator or arm.
  3. Select the appropriate Static Feedforward Sign for your closed-loop type.
  4. Increase Ks until just before the motor moves.
  5. If using velocity setpoints, increase Kv until the output velocity closely matches the velocity setpoints.
  6. Increase Kp until the output starts to oscillate around the setpoint.
  7. Increase Kd as much as possible without introducing jittering to the response.

  Suggested tuning procedure:
  1. Set elevator perpendicular to the ground (lay it on it's side)
  2. Tune kS (static friction)
  3. Stand robot back up
  4. Tune kG (gravity, elevator may need help to reach position; but must HOLD position without assistance)
  5. Tune kP
  6. Tune dD
  */

  /** Creates a new Elevator. */
  private final TalonFX m_leftElevator;
  private final TalonFX m_rightElevator;
  private TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

  private final MotionMagicVoltage m_goal = new MotionMagicVoltage(0);

  private double percentOfElevator = 0.9;

  // TODO: Move constants to Constants.java after tuning
  private static final double ELEV_CURRENT_LIMIT = 40.0;  // Stator current limit (not sure if needed)
  // https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/motion-magic.html
  private static final double ELEV_kG = 0.0; // output to overcome gravity (output)
  private static final double ELEV_kS = 0.0; // output to overcome static friction (output)
  private static final double ELEV_kV = 0.0; // output per unit of target velocity (output/rps)
  private static final double ELEV_kA = 0.0; // output per unit of target acceleration (output/(rps/s))
  private static final double ELEV_kP = 0.0; // output per unit of error in position (output/rotation)
  private static final double ELEV_kI = 0.0; // output per unit of integrated error in position (output/(rotation*s))
  private static final double ELEV_kD = 0.0; // output per unit of error in velocity (output/rps)
  private static final double ELEV_maxV = 80.0; // Maximum velocity (rps)
  private static final double ELEV_maxA = 160.0; // Maximum acceleration (rps/s)
  private static final double ELEV_maxJ = 1600.0; // Maximum jerk (rps/s/s)

  public Elevator() {
    m_leftElevator = new TalonFX(Constants.SwerveConstants.kLeftElevatorMotorPort);

    m_rightElevator = new TalonFX(Constants.SwerveConstants.kRightElevatorMotorPort);
    m_rightElevator.setControl(new Follower(m_leftElevator.getDeviceID(), true));  // set the right to follow the left (inverted)

    updateMotorConfig();
    
    m_leftElevator.setNeutralMode(NeutralModeValue.Brake);
    m_rightElevator.setNeutralMode(NeutralModeValue.Brake);

    m_leftElevator.setPosition(0);
  }

  private void updateMotorConfig(){
    // In Competition mode these values are not overwritten.
    double currentLimit = ELEV_CURRENT_LIMIT;
    double coeff_G = ELEV_kG;
    double coeff_S = ELEV_kS;
    double coeff_V = ELEV_kV;
    double coeff_A = ELEV_kA;
    double coeff_P = ELEV_kP;
    double coeff_I = ELEV_kI;
    double coeff_D = ELEV_kD;
    double coeff_maxV = ELEV_maxV;
    double coeff_maxA = ELEV_maxA;
    double coeff_maxJ = ELEV_maxJ;

    if (TUNE_PID_MODE) {
      // Check Shuffleboard for any updated values (use defaults on first run)
      currentLimit = SmartDashboard.getNumber("Elev Current Limit", ELEV_CURRENT_LIMIT);
      coeff_G = SmartDashboard.getNumber("Elev kGravity", ELEV_kG);
      coeff_S = SmartDashboard.getNumber("Elev kStaticFriction", ELEV_kS);
      coeff_V = SmartDashboard.getNumber("Elev kVelocity", ELEV_kV);
      coeff_A = SmartDashboard.getNumber("Elev kAcceleration", ELEV_kA);
      coeff_P = SmartDashboard.getNumber("Elev kProportion", ELEV_kP);
      coeff_I = SmartDashboard.getNumber("Elev kIntegral", ELEV_kI);
      coeff_D = SmartDashboard.getNumber("Elev kDifferential", ELEV_kD);
      coeff_maxV = SmartDashboard.getNumber("Elev kMaxVelocity", ELEV_maxV);
      coeff_maxA = SmartDashboard.getNumber("Elev kMaxAcceleration", ELEV_maxA);
      coeff_maxJ = SmartDashboard.getNumber("Elev kMaxJerk", ELEV_maxJ);
    }

    // Set the actual configuration now that values are set

    // Is this even necessary?  Default is 120A (adjust this after tuning max vel and acc)
    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.StatorCurrentLimit = currentLimit;

    elevatorConfig.Slot0.kG = coeff_G;
    elevatorConfig.Slot0.kS = coeff_S;
    elevatorConfig.Slot0.kV = coeff_V;
    elevatorConfig.Slot0.kA = coeff_A;
    elevatorConfig.Slot0.kP = coeff_P;
    elevatorConfig.Slot0.kI = coeff_I;
    elevatorConfig.Slot0.kD = coeff_D;

    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = coeff_maxV;
    elevatorConfig.MotionMagic.MotionMagicAcceleration = coeff_maxA;
    elevatorConfig.MotionMagic.MotionMagicJerk = coeff_maxJ;

    m_rightElevator.getConfigurator().apply(elevatorConfig);
    m_leftElevator.getConfigurator().apply(elevatorConfig);
  }

  public void setGoal(double targetPosition) {
    // m_leftElevator.setControl(position.withEnableFOC(false).withPosition(targetPosition));
    m_leftElevator.setControl(m_goal.withEnableFOC(false).withPosition(targetPosition));
  }

  public void stopRotate() {
    m_leftElevator.stopMotor();
  }

  public double getMotorPosition() {
    return m_leftElevator.getPosition().getValueAsDouble();
  }

  public boolean isAtSetpoint() {
    return Math.abs(m_leftElevator.getClosedLoopError().getValueAsDouble()) <= 1.0;
  }

  /**
   * @param presetNumber Preset of elevator to go to (0 - 1)
   */
  public void setElevatorPreset(double presetNumber) {
    percentOfElevator = presetNumber;
  }

  public double getElevatorPreset() {
    return percentOfElevator;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elev Position", m_leftElevator.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elev Velocity", m_leftElevator.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Elev Acceleration", m_leftElevator.getAcceleration().getValueAsDouble());
    SmartDashboard.putNumber("Elev Error", m_leftElevator.getClosedLoopError().getValueAsDouble());
    SmartDashboard.putBoolean("Elev At Setpoint", isAtSetpoint());
    SmartDashboard.putNumber("Elev Setpoint", percentOfElevator);
  }
}
