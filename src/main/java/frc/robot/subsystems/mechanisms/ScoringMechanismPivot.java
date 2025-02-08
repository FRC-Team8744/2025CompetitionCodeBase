// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ScoringMechanismPivot extends SubsystemBase {
  /** Creates a new ScoringMechanismPivot. */
  private final TalonFX m_scoringMechPivot;
  private final double startingPositionRotations = 0;
  private double minimumAngle = 0;
  private double maximumAngle = 110;
  private final PositionVoltage goalPosition = new PositionVoltage(startingPositionRotations);
  public ScoringMechanismPivot() {
    m_scoringMechPivot = new TalonFX(36);

    m_scoringMechPivot.getConfigurator().apply(Constants.scoringMechPivotConfig);
    m_scoringMechPivot.setNeutralMode(NeutralModeValue.Brake);
    m_scoringMechPivot.setPosition(startingPositionRotations);
  }

  public void algaeConfig(double angle) {
    if (angle < minimumAngle) {angle = minimumAngle;}
    if (angle > maximumAngle) {angle = maximumAngle;}
    m_scoringMechPivot.setControl(goalPosition.withEnableFOC(false).withSlot(0).withPosition(angle / 360));
  }

  public double getPositionAngle() {
    return m_scoringMechPivot.getPosition().getValueAsDouble() * 360;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
