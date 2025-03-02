// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ConstantsOffboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.RunElevator;
import frc.robot.commands.TeleopScore;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ScoringMechSensor;
import frc.robot.subsystems.alignment.AlignToClimb;
// import frc.robot.subsystems.mechanisms.AlgaeMechanism;
import frc.robot.subsystems.mechanisms.Climber;
import frc.robot.subsystems.mechanisms.CoralScoring;
import frc.robot.subsystems.mechanisms.Elevator;
import frc.robot.subsystems.mechanisms.Intake;
import frc.robot.subsystems.mechanisms.IntakePivot;
import frc.robot.subsystems.mechanisms.ScoringMechanismPivot;
import frc.robot.subsystems.vision.PhotonVisionGS;
import frc.robot.subsystems.vision.PhotonVisionGS2;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.RunIntake;
import frc.robot.commands.TeleopOuttake;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
@Logged
public class RobotContainer {
  // The robot's subsystems
  private PhotonVisionGS m_vision = new PhotonVisionGS();
  private PhotonVisionGS2 m_vision2 = new PhotonVisionGS2();
  private DriveSubsystem m_robotDrive = new DriveSubsystem(m_vision, m_vision2);
  private Elevator m_elevator = new Elevator();
  private Intake m_intake = new Intake();
  private IntakePivot m_intakePivot = new IntakePivot();
  private ScoringMechanismPivot m_scoringMechPivot = new ScoringMechanismPivot();
  private CoralScoring m_coral = new CoralScoring();
  private Climber m_climber = new Climber();
  // private AlgaeMechanism m_algae = new AlgaeMechanism();
  private ScoringMechSensor m_scoringMechSensor = new ScoringMechSensor();
  // The driver's controller
  private CommandXboxController m_driver = new CommandXboxController(OIConstants.kDriverControllerPort);
  private AutoCommandManager m_autoManager = new AutoCommandManager(m_elevator, m_intake, m_climber, m_coral, m_intakePivot, m_scoringMechPivot, m_vision, m_vision2, null, null, m_robotDrive, null,m_scoringMechSensor);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

  // Configure default commands
      m_robotDrive.setDefaultCommand(
          // The left stick controls translation of the robot.
          // Turning is controlled by the X axis of the right stick.
          new RunCommand(
              () ->
                  m_robotDrive.drive(
                      -m_driver.getLeftY()  * SwerveConstants.kMaxSpeedTeleop,
                      -m_driver.getLeftX()  * SwerveConstants.kMaxSpeedTeleop,
                      m_driver.getRightX() * ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND,
                      true),
              m_robotDrive));
    // m_autoChooser = AutoBuilder.buildAutoChooser();  // Default auto will be 'Commands.none()'

    // SmartDashboard.putData("Auto Mode", m_autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  
  private void configureButtonBindings() {
    m_driver.back().onTrue(Commands.runOnce (() -> m_robotDrive.zeroGyro()));
    m_driver.rightStick()
    .toggleOnTrue(Commands.runOnce(() -> m_robotDrive.isAutoRotate = m_robotDrive.isAutoRotate == RotationEnum.STRAFEONTARGET ? RotationEnum.NONE : RotationEnum.STRAFEONTARGET));
    // m_driver.leftTrigger()
    // .toggleOnTrue(Commands.runOnce(() -> m_robotDrive.isAutoRotate = m_robotDrive.isAutoRotate == RotationEnum.ALIGNTOCLIMB ? RotationEnum.NONE : RotationEnum.ALIGNTOCLIMB));
    m_driver.leftTrigger()
    // .whileTrue(Commands.runOnce(() -> m_elevator.elevatorConfig.CurrentLimits.StatorCurrentLimit = 40)
    // .alongWith(Commands.runOnce(() -> m_elevator.position.withSlot(0)))
    // .alongWith(Commands.runOnce(() -> m_elevator.elevatorSlot0 = true))
    .whileTrue(new RunElevator(m_elevator).alongWith(Commands.runOnce(() -> m_scoringMechPivot.rotatePivot(m_scoringMechPivot.scoringMechGoalAngle)).onlyWhile((() -> m_elevator.getMotorPosition() >= ((327 * m_elevator.getElevatorPreset()) * .75)))))
    
    // .whileFalse(Commands.runOnce(() -> m_elevator.elevatorConfig.CurrentLimits.StatorCurrentLimit = 5)
    // .alongWith(Commands.runOnce(() -> m_elevator.position.withSlot(1)))
    // .alongWith(Commands.runOnce(() -> m_elevator.elevatorSlot0 = false))
    .whileFalse(Commands.runOnce(() -> m_scoringMechPivot.rotatePivot(0)).alongWith(Commands.runOnce(() -> m_elevator.setGoal(0)).onlyWhile((() -> m_scoringMechPivot.getPositionAngle() >= -20))));

    m_driver.rightTrigger()
    .whileTrue(new RunIntake(m_intake, m_intakePivot, m_coral, m_scoringMechSensor));

    m_driver.y()
    .whileTrue(new TeleopScore(m_coral).finallyDo((() -> m_robotDrive.isAutoYSpeed = false)));
    
    m_driver.rightBumper()
    .toggleOnTrue(Commands.runOnce(() -> m_robotDrive.rightPoint = false).andThen(Commands.runOnce(() -> m_robotDrive.isAutoYSpeed = true)));
    
    m_driver.leftBumper()
    .toggleOnTrue(Commands.runOnce(() -> m_robotDrive.rightPoint = true).andThen(Commands.runOnce(() -> m_robotDrive.isAutoYSpeed = true)));

    m_driver.a()
    .whileTrue(new TeleopOuttake(m_intake, m_intakePivot, m_coral));

    m_driver.x()
    .whileTrue(Commands.runOnce(() -> m_intake.runIndexer(.3)));

    m_driver.b()
    .whileTrue(Commands.runOnce(() -> m_robotDrive.isAutoYSpeed = false));

    m_driver.pov(0)
    .whileTrue(Commands.runOnce(() -> m_elevator.setElevatorPreset(.9)).alongWith(Commands.runOnce(() -> m_scoringMechPivot.scoringMechGoalAngle = -200)));
    m_driver.pov(90)
    .whileTrue(Commands.runOnce(() -> m_elevator.setElevatorPreset(.53)).alongWith(Commands.runOnce(() -> m_scoringMechPivot.scoringMechGoalAngle = -60)));
    m_driver.pov(180)
    .whileTrue(Commands.runOnce(() -> m_elevator.setElevatorPreset(.25)).alongWith(Commands.runOnce(() -> m_scoringMechPivot.scoringMechGoalAngle = -60)));
    m_driver.pov(270)
    .whileTrue(Commands.runOnce(() -> m_elevator.setElevatorPreset(.33)).alongWith(Commands.runOnce(() -> m_scoringMechPivot.scoringMechGoalAngle = -60)));
  } 

  public Command getAutonomousCommand() {
    Command autoCommand = m_autoManager.getAutoManagerSelected();
    return autoCommand;
  }
}