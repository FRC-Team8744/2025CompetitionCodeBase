// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.mechanisms.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunElevator extends Command {
  /** Creates a new RunKraken. */
  private final Elevator m_elevator;
  private double motorPosition;
  public RunElevator(Elevator ele) {
    m_elevator = ele;
    addRequirements(m_elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motorPosition = m_elevator.getMotorPosition();

    SmartDashboard.putBoolean("Command run", true);
    SmartDashboard.putNumber("Elevator Motor Position", motorPosition);

    m_elevator.setGoal(16.35 * Constants.ELEVATOR_GEARING * m_elevator.getElevatorPreset()); // 327
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.isAtSetpoint();
  }
}
