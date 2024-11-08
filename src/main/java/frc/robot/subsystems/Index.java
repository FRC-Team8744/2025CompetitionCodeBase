// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Index extends SubsystemBase {
  /** Creates a new Index. */
  public SparkMax IndexMotor = new SparkMax(18, MotorType.kBrushless);
  public SparkBaseConfig IndexConfig = new SparkMaxConfig().smartCurrentLimit(20);
  public Index() {
    IndexMotor.configure(IndexConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void runIndex() {
    IndexMotor.set(.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
