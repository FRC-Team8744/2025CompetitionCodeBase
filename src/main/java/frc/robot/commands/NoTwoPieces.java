// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.Intake;
import frc.robot.subsystems.mechanisms.IntakePivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class NoTwoPieces extends Command {
  /** Creates a new NoTwoPieces. */
  private Intake m_intake;
  private IntakePivot m_intakePivot;
  public NoTwoPieces(Intake in, IntakePivot inp) {
    m_intake = in;
    addRequirements(m_intake);
    m_intakePivot = inp;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.runIndexer(-0.5, 0.5);
    m_intake.runIntake(-0.6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intakePivot.getPositionAngle() >= -20;
  }
}
