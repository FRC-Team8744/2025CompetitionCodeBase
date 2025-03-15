// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeMechanism extends SubsystemBase {
  private final SparkMax m_algaeMotor;
  private final SparkBaseConfig algaeConfig = new SparkMaxConfig().smartCurrentLimit(20);
  public boolean intakingAlgae = false;
  /** Creates a new AlgaeMechanism. */
  public AlgaeMechanism() {
    m_algaeMotor = new SparkMax(Constants.SwerveConstants.kAlgaeScoringMotorPort, MotorType.kBrushless);

    m_algaeMotor.configure(algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void intakeAlgae(double speed) {
    m_algaeMotor.set(speed);
  }

  public void scoreAlgae(double speed) {
    m_algaeMotor.set(-speed);
  }

  public void passiveAlgae() {
    m_algaeMotor.set(0.05);
  }

  public void stopMotor() {
    m_algaeMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!intakingAlgae && Constants.scoringMode == "Algae") {
        passiveAlgae();
    }
  }
}