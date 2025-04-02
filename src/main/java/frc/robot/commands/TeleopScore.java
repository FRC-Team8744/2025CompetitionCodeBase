// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ScoringMechSensor;
import frc.robot.subsystems.mechanisms.AlgaeMechanism;
import frc.robot.subsystems.mechanisms.CoralScoring;
import frc.robot.subsystems.mechanisms.Elevator;
import frc.robot.subsystems.mechanisms.Intake;
import frc.robot.subsystems.mechanisms.IntakePivot;
import frc.robot.subsystems.mechanisms.ScoringMechanismPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopScore extends Command{
  /** Creates a new TeleopIntake. */
  private final CoralScoring m_coral;
  private final Elevator m_elevator;
  private final Intake m_intake;
  private final IntakePivot m_intakePivot;
  private final ScoringMechSensor m_scoringMechSensor;
  private final AlgaeMechanism m_algae;
  private final ScoringMechanismPivot m_scoringMechPivot;
  private final DriveSubsystem m_drive;
  public TeleopScore(CoralScoring co, Elevator ele, Intake in, IntakePivot inp, ScoringMechSensor sms, AlgaeMechanism alg, ScoringMechanismPivot smp, DriveSubsystem dr) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_coral = co;
    addRequirements(m_coral);
    m_elevator = ele;
    m_intake = in;
    addRequirements(m_intake);
    m_intakePivot = inp;
    addRequirements(m_intakePivot);
    m_scoringMechSensor = sms;
    m_algae = alg;
    addRequirements(m_algae);
    m_scoringMechPivot = smp;
    m_drive = dr;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Constants.scoringMode == "Coral") {
      m_coral.runCoralMotor(-.4);  
    }
    else if (Constants.scoringMode == "Algae") {
      m_algae.scoreAlgae(0.4);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coral.stopMotor();
    m_intake.stopBoth();
    m_algae.stopMotor();
    if (Constants.scoringMode == "Algae") {
      m_scoringMechPivot.rotatePivot(0);
      m_elevator.rotate(0);
    }
    m_drive.isDrivingSlow = false;
    // m_intakePivot.intakeDown(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
