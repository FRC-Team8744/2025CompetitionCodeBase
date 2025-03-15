// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.Elevator;
import frc.robot.subsystems.mechanisms.IntakePivot;
import frc.robot.subsystems.mechanisms.ScoringMechanismPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ResetEncoders extends Command {
  /** Creates a new ResetEncoders. */
  private Elevator m_elevator;
  private ScoringMechanismPivot m_scoringMechPivot;
  private IntakePivot m_intakePivot;
  private Timer m_timer = new Timer();
  public ResetEncoders(Elevator ele, ScoringMechanismPivot scp, IntakePivot inp) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = ele;
    addRequirements(m_elevator);
    m_scoringMechPivot = scp;
    addRequirements(m_scoringMechPivot);
    m_intakePivot = inp;
    addRequirements(m_intakePivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    m_scoringMechPivot.goDown();
    // m_intakePivot.goDown();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.hasElapsed(0.5)) {
      m_elevator.goDown();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_scoringMechPivot.resetEncoder();
    m_elevator.resetElevatorPosition();
    // m_intakePivot.resetIntakePivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
