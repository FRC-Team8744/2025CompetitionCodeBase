// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ScoringMechSensor;
import frc.robot.subsystems.mechanisms.CoralScoring;
import frc.robot.subsystems.mechanisms.Elevator;
import frc.robot.subsystems.mechanisms.Intake;
import frc.robot.subsystems.mechanisms.IntakePivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScore extends Command{
  /** Creates a new TeleopIntake. */
  private final CoralScoring m_coral;
  private final Elevator m_elevator;
  private final Intake m_intake;
  private final IntakePivot m_intakePivot;
  private final ScoringMechSensor m_scoringMechSensor;
  private final Timer m_timer = new Timer();
  public AutoScore(CoralScoring co, Elevator ele, Intake in, IntakePivot inp, ScoringMechSensor sms) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_coral = co;
    addRequirements(m_coral);
    m_elevator = ele;
    m_intake = in;
    addRequirements(m_intake);
    m_intakePivot = inp;
    addRequirements(m_intakePivot);
    m_scoringMechSensor = sms;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_coral.runCoralMotor(-.4);
    // m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_scoringMechSensor.getScoringSensor()) {
      m_timer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coral.stopMotor();
    m_intake.stopBoth();
    m_timer.reset();
    // m_intakePivot.intakeDown(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.hasElapsed(0.3)) {
      m_timer.stop();
      return true;
    }
    else {
      return false;
    }
  }
}
