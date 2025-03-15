// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RotationEnum;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ScoringMechSensor;
import frc.robot.subsystems.mechanisms.Elevator;
import frc.robot.subsystems.mechanisms.ScoringMechanismPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorToScoreAuto extends Command {
  /** Creates a new ElevatorToScore. */
  private Elevator m_elevator;
  private DriveSubsystem m_robotDrive;
  private double motorPosition;
  private ScoringMechanismPivot m_scoringMechPivot;
  private ScoringMechSensor m_scoringMechSensor;
  private boolean toggle = true;
  private boolean rotateToggle = true;
  public ElevatorToScoreAuto(Elevator ele, DriveSubsystem dr, ScoringMechanismPivot scp, ScoringMechSensor sms) {
    m_elevator = ele;
    addRequirements(m_elevator);
    m_robotDrive = dr;
    m_scoringMechPivot = scp;
    addRequirements(m_scoringMechPivot);
    m_scoringMechSensor = sms;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    toggle = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_robotDrive.isAutoRotate = RotationEnum.STRAFEONTARGET;

    motorPosition = m_elevator.getMotorPosition();

    if (Constants.scoringMode == "Coral") {
      if (!m_scoringMechSensor.getScoringSensor()) {
        m_elevator.rotate(16.35 * Constants.ELEVATOR_GEARING * Constants.percentOfElevator); // 327
        if (m_elevator.getMotorPosition() >= ((16.35 * Constants.ELEVATOR_GEARING * Constants.percentOfElevator) * .50) && toggle) {
          m_robotDrive.isAutoYSpeed = true;
          m_robotDrive.isAutoXSpeed = true;
          m_scoringMechPivot.rotatePivot(Constants.scoringMechGoalAngle);
          toggle = false;
        }
        m_robotDrive.driveRobotRelative(new ChassisSpeeds());
      }
    }
    else if (Constants.scoringMode == "Algae") {
      if (m_robotDrive.autoRotateSpeed == 0) {
        m_elevator.rotate(16.35 * Constants.ELEVATOR_GEARING * Constants.percentOfElevatorAlgae);
        if (m_elevator.getMotorPosition() >= ((16.35 * Constants.ELEVATOR_GEARING * Constants.percentOfElevatorAlgae) * 0.50) && toggle) {
          m_robotDrive.isAutoYSpeed = true;
          m_robotDrive.isAutoXSpeed = true;
          m_scoringMechPivot.rotatePivot(Constants.scoringMechGoalAngleAlgae);
          toggle = false;
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_scoringMechPivot.rotatePivot(0);
    // m_elevator.rotate(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_elevator.isAtSetpoint() && m_robotDrive.m_alignToPoleX.hasReachedX && m_robotDrive.m_alignToPole.hasReachedY) {
      return true;
    }
    else {
      return false;
    }
  }
}