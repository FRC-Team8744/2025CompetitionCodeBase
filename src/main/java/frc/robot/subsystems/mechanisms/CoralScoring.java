// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class CoralScoring extends SubsystemBase {
  /** Creates a new CoralScoring. */
  private final SparkMax m_coralMotor;
  private final SparkBaseConfig coralConfig = new SparkMaxConfig().smartCurrentLimit(40).idleMode(IdleMode.kBrake);
  public CoralScoring() {
    m_coralMotor = new SparkMax(Constants.SwerveConstants.kCoralScoringMotorPort, MotorType.kBrushless);

    m_coralMotor.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  // Runs the motor with the speed set
  public void runCoralMotor(double speed) {
    m_coralMotor.set(speed);
  }

  // Stops the motor
  public void stopMotor() {
    m_coralMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
