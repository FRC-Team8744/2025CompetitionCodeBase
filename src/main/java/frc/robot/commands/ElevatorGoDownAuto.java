// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.Elevator;
import frc.robot.subsystems.mechanisms.ScoringMechanismPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorGoDownAuto extends Command {
  /** Creates a new ElevatorGoDownAuto. */
  private Elevator m_elevator;
  private ScoringMechanismPivot m_scoringMechPivot;
  public ElevatorGoDownAuto(Elevator ele, ScoringMechanismPivot scp) {
    m_elevator = ele;
    addRequirements(m_elevator);
    m_scoringMechPivot = scp;
    addRequirements(m_scoringMechPivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_scoringMechPivot.rotatePivot(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_scoringMechPivot.getPositionAngle()) <= 20) {
      m_elevator.rotate(0);
    }
    SmartDashboard.putBoolean("Elevator go down command work", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.stopRotate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_elevator.isAtSetpoint() && m_scoringMechPivot.isAtSetpoint()) {
      return true;
    }
    else {
      return false;
    }
  }
}
