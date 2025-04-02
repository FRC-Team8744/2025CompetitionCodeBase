// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RotationEnum;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.mechanisms.AlgaeMechanism;
import frc.robot.subsystems.mechanisms.Elevator;
import frc.robot.subsystems.mechanisms.ScoringMechanismPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorToScore extends Command {
  /** Creates a new ElevatorToScore. */
  private Elevator m_elevator;
  private DriveSubsystem m_robotDrive;
  private double motorPosition;
  private ScoringMechanismPivot m_scoringMechPivot;
  private boolean toggle = true;
  private AlgaeMechanism m_algae;
  public ElevatorToScore(Elevator ele, DriveSubsystem dr, ScoringMechanismPivot scp, AlgaeMechanism alg) {
    m_elevator = ele;
    addRequirements(m_elevator);
    m_robotDrive = dr;
    m_scoringMechPivot = scp;
    addRequirements(m_scoringMechPivot);
    m_algae = alg;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Constants.visionElevator) {
      if (Constants.scoringMode == "Algae" && Constants.algaeScoringLevel == "Net") {
        m_robotDrive.isAutoRotate = RotationEnum.NONE;
      }
      else {
        m_robotDrive.isAutoRotate = RotationEnum.STRAFEONTARGET;
      }
    }
    toggle = true;
    m_robotDrive.isDrivingSlow = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motorPosition = m_elevator.getMotorPosition();

    // if (Constants.scoringMode == "Coral") {
    //   if (m_robotDrive.autoRotateSpeed == 0) {
    //     m_elevator.rotate(16.35 * Constants.ELEVATOR_GEARING * Constants.percentOfElevator); // 327
    //     m_scoringMechPivot.rotatePivot(Constants.scoringMechGoalAngle);
    //     if (m_elevator.getMotorPosition() >= ((16.35 * Constants.ELEVATOR_GEARING * Constants.percentOfElevator) * .50) && toggle) {
    //       if (Constants.visionElevator) {
    //         m_robotDrive.isAutoYSpeed = true;
    //         m_robotDrive.isAutoXSpeed = true;
    //       }
    //       toggle = false;
    //     }
    //   }
    // }
    if (Constants.scoringMode == "Coral") {
      // SmartDashboard.putNumber("AutoRotateSpeed", m_robotDrive.autoRotateSpeed);
      // if (m_robotDrive.autoRotateSpeed == 0 && m_robotDrive.isAutoRotate == RotationEnum.STRAFEONTARGET) {
        if (Constants.visionElevator) {
          m_robotDrive.isAutoYSpeed = true;
          m_robotDrive.isAutoXSpeed = true;
          if (Math.abs(m_robotDrive.m_alignToPoleX.xOffset) <= 1.5 ) {
            m_elevator.rotate(16.35 * Constants.ELEVATOR_GEARING * Constants.percentOfElevator); // 327
            if (Constants.scoringLevel == "L4") {
              double movingScoringMechPivotAngle = 1681.52953862 - 1613.42103817 * Math.log(m_robotDrive.m_alignToPoleX.robotX);
              // SmartDashboard.putNumber("GoalScoringMechAngle", movingScoringMechPivotAngle);
              m_scoringMechPivot.rotatePivot(movingScoringMechPivotAngle);
              // m_scoringMechPivot.rotatePivot(Constants.scoringMechGoalAngle);
            } else {
              m_scoringMechPivot.rotatePivot(Constants.scoringMechGoalAngle);
            }
            }
          } else {
            m_elevator.rotate(16.35 * Constants.ELEVATOR_GEARING * Constants.percentOfElevator); // 327
            m_scoringMechPivot.rotatePivot(Constants.scoringMechGoalAngle);
          // }
        }
      }
    else if (Constants.scoringMode == "Algae") {
      if (m_robotDrive.autoRotateSpeed == 0) {
        m_algae.intakeAlgae(0.2);
        m_elevator.rotate(16.35 * Constants.ELEVATOR_GEARING * Constants.percentOfElevatorAlgae);
        if (m_elevator.getMotorPosition() >= ((16.35 * Constants.ELEVATOR_GEARING * Constants.percentOfElevatorAlgae) * 0.50) && toggle) {
          if (Constants.visionElevator && (Constants.algaeScoringLevel == "L2" || Constants.algaeScoringLevel == "L3")) {
            m_robotDrive.isAutoYSpeed = true;
            m_robotDrive.isAutoXSpeed = true;
          }
          m_scoringMechPivot.rotatePivot(Constants.scoringMechGoalAngleAlgae);
          toggle = false;
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (Constants.scoringMode == "Coral") {
      m_elevator.rotate(0);
      m_scoringMechPivot.rotatePivot(0);
    }
    if (Constants.scoringMode == "Algae") {
      m_algae.intakeAlgae(0.2);
      m_robotDrive.isAutoYSpeed = false;
      m_robotDrive.isAutoXSpeed = false;
      m_robotDrive.isAutoRotate = RotationEnum.NONE;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}