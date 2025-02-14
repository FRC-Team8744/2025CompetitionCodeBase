// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.Elevator;
// import frc.robot.subsystems.mechanisms.Intake;
import frc.robot.subsystems.mechanisms.ScoringMechanismPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunElevator extends Command {

    private final Elevator m_elevator;
    private final ScoringMechanismPivot m_scoringPivot;
    private double motorPosition;
  /** Creates a new RunElevator. */
  public RunElevator(Elevator ele, ScoringMechanismPivot smp) {
    
    m_elevator = ele;
    addRequirements(m_elevator);
    m_scoringPivot = smp;
    addRequirements(m_scoringPivot);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  motorPosition = m_elevator.motorPosition();

  m_elevator.rotate(327 * .5);
  }
  // 327
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.isAtSetpoint();
  }
}
