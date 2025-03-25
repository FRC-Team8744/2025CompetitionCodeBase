// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringMechSensor;
import frc.robot.subsystems.mechanisms.Intake;
import frc.robot.subsystems.mechanisms.IntakePivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class NoTwoPiecesAuto extends Command {
  /** Creates a new NoTwoPieces. */
  private Intake m_intake;
  private IntakePivot m_intakePivot;
  private ScoringMechSensor m_scoringMechSensor;
  private Timer timer = new Timer();
  public NoTwoPiecesAuto(Intake in, IntakePivot inp, ScoringMechSensor sms) {
    m_intake = in;
    // addRequirements(m_intake);
    m_intakePivot = inp;
    m_scoringMechSensor = sms;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_scoringMechSensor.getScoringSensor()) {
      m_intake.runIndexer(-0.5, 0.5);
      m_intake.runIntake(-0.6);
      timer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopBoth();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(2);
  }
}
