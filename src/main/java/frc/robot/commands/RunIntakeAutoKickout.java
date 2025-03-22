// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ScoringMechSensor;
import frc.robot.subsystems.mechanisms.AlgaeMechanism;
import frc.robot.subsystems.mechanisms.CoralScoring;
import frc.robot.subsystems.mechanisms.Intake;
import frc.robot.subsystems.mechanisms.IntakePivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunIntakeAutoKickout extends Command {
  /** Creates a new TeleopIntake. */
  private final Intake m_intake;
  private final IntakePivot m_intakePivot;
  private final CoralScoring m_coral;
  private final ScoringMechSensor m_sensor;
  private final AlgaeMechanism m_algae;
  private final ElevatorToScoreAuto m_elevatorToScore;
  private final Timer m_timer = new Timer();
  private final Timer m_stopTimer = new Timer();
  private final Timer m_newTimer = new Timer();
  private final NoTwoPieces m_noTwoPieces;
  private boolean toggle = true;
  public RunIntakeAutoKickout(Intake in, IntakePivot inp, CoralScoring co, ScoringMechSensor scp, AlgaeMechanism alg, ElevatorToScoreAuto ets, NoTwoPieces ntp) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = in;
    addRequirements(m_intake);
    m_intakePivot = inp;
    addRequirements(m_intakePivot);
    m_coral = co;
    addRequirements(m_coral);
    m_sensor = scp;
    m_elevatorToScore = ets;
    m_algae = alg;
    addRequirements(m_algae);
    m_noTwoPieces = ntp;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    SmartDashboard.putBoolean("Started Intake initialize", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_newTimer.start();
    if (Constants.scoringMode == "Coral") {
      if (!m_timer.hasElapsed(2)) {
        if (m_sensor.getScoringSensor()) {
          m_intake.runIndexer(.5, -0.5);
          m_intake.runIntake(.6);
          m_coral.runCoralMotor(-.075);
          m_intakePivot.intakeDown(-3393);
          m_stopTimer.reset();
        }
      }
      else {
        m_intake.stopBoth();
        m_intake.stopIndexer();
        m_coral.stopMotor();
        m_stopTimer.start();
        if (m_stopTimer.hasElapsed(0.2)) {
          m_timer.restart();
        }
      }
    }
    else if (Constants.scoringMode == "Algae") {
      if (Constants.algaeScoringLevel == "L2" || Constants.algaeScoringLevel == "L3") {
        m_algae.intakeAlgae(0.4);
        m_elevatorToScore.schedule();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coral.stopMotor();
    m_intake.stopBoth();
    m_newTimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_newTimer.hasElapsed(0.15)) {
      return true;
    }
    return !m_sensor.getScoringSensor();
  }
}