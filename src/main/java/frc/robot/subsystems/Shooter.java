// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public SparkMax leftShooter = new SparkMax(14, MotorType.kBrushless);
  public SparkMax rightShooter = new SparkMax(15, MotorType.kBrushless);
  public SparkBaseConfig shooterConfig = new SparkMaxConfig().smartCurrentLimit(20);

  public Shooter() {
    leftShooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rightShooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void shootRing () {
    rightShooter.set(-1.0);
    leftShooter.set(1.0);
  }

  public void stopShooter() {
    rightShooter.set(0);
    leftShooter.set(0);
  }

  public void shooterIntakeRing() {
    rightShooter.set(-1);
    leftShooter.set(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
