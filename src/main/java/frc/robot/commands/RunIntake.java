// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.ScoringMechSensor;
import frc.robot.subsystems.mechanisms.AlgaeMechanism;
import frc.robot.subsystems.mechanisms.CoralScoring;
import frc.robot.subsystems.mechanisms.Intake;
import frc.robot.subsystems.mechanisms.IntakePivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunIntake extends Command {
  /** Creates a new TeleopIntake. */
  private final Intake m_intake;
  private final IntakePivot m_intakePivot;
  private final CoralScoring m_coral;
  private final ScoringMechSensor m_sensor;
  private final AlgaeMechanism m_algae;
  private final ElevatorToScore m_elevatorToScore;
  public RunIntake(LEDS led, Intake in, IntakePivot inp, CoralScoring co, ScoringMechSensor scp, AlgaeMechanism alg, ElevatorToScore ets) {
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
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Constants.scoringMode == "Coral") {
      m_intake.runIndexer(.5, -0.5);
      m_intake.runIntake(.6);
      m_coral.runCoralMotor(-.05);
      m_intakePivot.intakeDown(-3393.45703125);
    }
    else if (Constants.scoringMode == "Algae") {
      if (Constants.algaeScoringLevel == "L2" || Constants.algaeScoringLevel == "L3") {
        m_algae.intakeAlgae(0.4);
        m_elevatorToScore.schedule();
      }
    }
    m_leds.SetSegmentByIntakeMech(Color.kRed, 50);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coral.stopMotor();
    m_intake.stopBoth();
    m_coral.stopMotor();
    m_intake.stopBoth();
    m_leds.SetSegmentByIntakeMech(Color.kGreen, 50);
    boolean m_return = !m_sensor.getScoringSensor();
    if(!m_return){
      m_leds.SetSegmentByIntakeMech(Color.kBlue, 50);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_sensor.getScoringSensor();
  }
}