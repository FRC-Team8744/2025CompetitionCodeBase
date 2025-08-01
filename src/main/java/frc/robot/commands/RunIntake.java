// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ColorInterface;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.ScoringMechSensor;
import frc.robot.subsystems.mechanisms.AlgaeMechanism;
import frc.robot.subsystems.mechanisms.CoralScoring;
import frc.robot.subsystems.mechanisms.Intake;
import frc.robot.subsystems.mechanisms.IntakePivot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunIntake extends Command {
  /** Creates a new TeleopIntake. */
  private final LEDS m_leds;
  private final Intake m_intake;
  private final IntakePivot m_intakePivot;
  private final CoralScoring m_coral;
  private final ScoringMechSensor m_sensor;
  private final AlgaeMechanism m_algae;
  private final NoTwoPieces m_noTwoPieces;
  private final ElevatorToIntakeAlgae m_elevatorToIntakeAlgae;
  private Timer m_timer = new Timer();
  public RunIntake(LEDS led, Intake in, IntakePivot inp, CoralScoring co, ScoringMechSensor scp, AlgaeMechanism alg, ElevatorToIntakeAlgae eia, NoTwoPieces ntp) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = in;
    addRequirements(m_intake);
    m_intakePivot = inp;
    addRequirements(m_intakePivot);
    m_coral = co;
    addRequirements(m_coral);
    m_sensor = scp;
    m_algae = alg;
    addRequirements(m_algae);
    m_noTwoPieces = ntp;
    m_elevatorToIntakeAlgae = eia;
    m_leds = led;
    addRequirements(m_leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if (Constants.scoringMode == "Coral") {
      m_intake.runIndexer(.5, -0.5);
      m_intake.runIntake(.6);
      m_coral.runCoralMotor(-.5);
      m_intakePivot.intakeDown(-3393);
    // }
    // else if (Constants.scoringMode == "Algae") {
    //   m_leds.SetSegmentByIntakeMech(ColorInterface.Algae, 50);
    //   if (Constants.algaeScoringLevel == "L2" || Constants.algaeScoringLevel == "L3") {
    //     m_algae.intakeAlgae(0.2);
    //     m_elevatorToIntakeAlgae.schedule();
    //     m_algae.intakingAlgae = true;
    //   }
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_timer
    // if(Constants.scoringMode == "Algae"){
      
    // } else if (Constants.scoringMode == "Coral"){
      m_leds.SetSegmentByIntakeMech(Color.kRed, 50);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coral.stopMotor();
    m_intake.stopBoth();
    // if (Constants.scoringMode == "Coral") {
      // m_intakePivot.intakeDown(0);
      m_noTwoPieces.schedule();
    // }
    m_algae.intakingAlgae = false;
    
    boolean m_return = m_sensor.getScoringSensor();
    if(m_return){
      m_leds.SetSegmentByIntakeMech(Color.kBlue, 50);
    } else {
      m_leds.SetSegmentByIntakeMech(Color.kGreen, 50);
    }
   
    // if (Constants.scoringMode == "Algae") {
    //   m_elevatorToIntakeAlgae.end(false);
    // }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Constants.sensorMode) {
      return !m_sensor.getScoringSensor();
    } else {
      return false;
    }
  }
}