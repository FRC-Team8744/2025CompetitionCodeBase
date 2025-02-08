// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private SparkMax m_indexMotor;
  private SparkMax m_intakeMotor;
  private SparkBaseConfig indexerConfg = new SparkMaxConfig().smartCurrentLimit(20);
  private SparkBaseConfig intakeConfig = new SparkMaxConfig().smartCurrentLimit(20);
  public Intake() {
    m_indexMotor = new SparkMax(25, MotorType.kBrushless);
    m_intakeMotor = new SparkMax(26, MotorType.kBrushless);

    m_indexMotor.configure(indexerConfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void runIndexer(double speed) {
    m_indexMotor.set(speed);
  }

  public void runIntake(double speed) {
    m_intakeMotor.set(speed);
  }

  public void stopIndexer() {
    m_indexMotor.stopMotor();
  }

  public void stopIntake() {
    m_intakeMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
