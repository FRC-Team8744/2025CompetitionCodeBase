// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringMechSensor;
import frc.robot.subsystems.mechanisms.CoralScoring;
import frc.robot.subsystems.mechanisms.Elevator;
import frc.robot.subsystems.mechanisms.Intake;
import frc.robot.subsystems.mechanisms.IntakePivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopIntake extends Command {
  /** Creates a new TeleopIntake. */
  private final Intake m_intake;
  private final IntakePivot m_intakePivot;
  private final CoralScoring m_coral;
  private final ScoringMechSensor m_sensor;
  public TeleopIntake(Intake in, IntakePivot inp, CoralScoring co, ScoringMechSensor se) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = in;
    addRequirements(m_intake);
    m_intakePivot = inp;
    addRequirements(m_intakePivot);
    m_coral = co;
    addRequirements(m_coral);
    m_sensor = se;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.runIntakeAndIndexer(.4);
    m_coral.runCoralMotor(-.4);
    m_intakePivot.intakeDown(-4000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopBoth();
    m_coral.stopMotor();
    m_intakePivot.intakeDown(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_sensor.getScoringSensor();
  }
}
