// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class TunePID extends SubsystemBase {
  private ShuffleboardTab tab;
  private GenericEntry kG;
  private GenericEntry kS;
  private GenericEntry kV;
  private GenericEntry kA;
  private GenericEntry kP;
  private GenericEntry kI;
  private GenericEntry kD;
  private GenericEntry maxV;
  private GenericEntry maxA;
  private GenericEntry maxJ;
  private GenericEntry limitI;

  private TalonFXConfiguration config;

  /** Creates a new TunePID. */
  public TunePID(TalonFXConfiguration startConfig) {
    config = startConfig;

    // Create tab for debug info
    tab = Shuffleboard.getTab("TunePID");
    kG = tab.add("kG", config.Slot0.kG).getEntry();
    kS = tab.add("kS", config.Slot0.kS).getEntry();
    kV = tab.add("kV", config.Slot0.kV).getEntry();
    kA = tab.add("kA", config.Slot0.kA).getEntry();
    kP = tab.add("kP", config.Slot0.kP).getEntry();
    kI = tab.add("kI", config.Slot0.kI).getEntry();
    kD = tab.add("kD", config.Slot0.kD).getEntry();
    maxV = tab.add("maxV", config.MotionMagic.MotionMagicCruiseVelocity).getEntry();
    maxA = tab.add("maxA", config.MotionMagic.MotionMagicAcceleration).getEntry();
    maxJ = tab.add("maxJ", config.MotionMagic.MotionMagicJerk).getEntry();
    limitI = tab.add("Current Limit", config.CurrentLimits.StatorCurrentLimit)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 80)).getEntry();
  }

  public TalonFXConfiguration getConfig(){
    double coeff_G = kG.getDouble(config.Slot0.kG);
    if (coeff_G >= 0) config.Slot0.kG = coeff_G;

    double coeff_S = kS.getDouble(config.Slot0.kS);
    if (coeff_S >= 0) config.Slot0.kS = coeff_S;

    double coeff_V = kV.getDouble(config.Slot0.kV);
    if (coeff_V >= 0) config.Slot0.kV = coeff_V;

    double coeff_A = kA.getDouble(config.Slot0.kA);
    if (coeff_A >= 0) config.Slot0.kA = coeff_A;

    double coeff_P = kP.getDouble(config.Slot0.kP);
    if (coeff_P >= 0) config.Slot0.kP = coeff_P;

    double coeff_I = kI.getDouble(config.Slot0.kI);
    if (coeff_I >= 0) config.Slot0.kI = coeff_I;

    double coeff_D = kD.getDouble(config.Slot0.kD);
    if (coeff_D >= 0) config.Slot0.kD = coeff_D;

    double coeff_maxV = maxV.getDouble(config.MotionMagic.MotionMagicCruiseVelocity);
    if (coeff_maxV >= 0) config.MotionMagic.MotionMagicCruiseVelocity = coeff_maxV;

    double coeff_maxA = maxA.getDouble(config.MotionMagic.MotionMagicAcceleration);
    if (coeff_maxA >= 0) config.MotionMagic.MotionMagicAcceleration = coeff_maxA;

    double coeff_maxJ = maxJ.getDouble(config.MotionMagic.MotionMagicJerk);
    if (coeff_maxJ >= 0) config.MotionMagic.MotionMagicJerk = coeff_maxJ;

    double currentLimit = limitI.getDouble(config.CurrentLimits.StatorCurrentLimit);
    if (currentLimit < 80) config.CurrentLimits.StatorCurrentLimit = currentLimit;

    return config;
}

  @Override
  public void periodic() {
  }
}
