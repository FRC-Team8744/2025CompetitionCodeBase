// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.mechanisms.Elevator;
import frc.robot.subsystems.mechanisms.ScoringMechanismPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoLineUpFake extends Command {
  /** Creates a new AutoLineUp. */
  private final Elevator m_elevator;
  private final DriveSubsystem m_driveBase;
  private final ScoringMechanismPivot m_scoringMechPivot;
  public AutoLineUpFake(Elevator ele, DriveSubsystem dri, ScoringMechanismPivot scp) {
    m_elevator = ele;
    addRequirements(m_elevator);
    m_driveBase = dri;
    m_scoringMechPivot = scp;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
