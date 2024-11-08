// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ConstantsOffboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Index;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.pathplanner.lib.auto.AutoBuilder;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private Index m_index = new Index();
  // The driver's controller
  DriveSubsystem m_robotDrive = new DriveSubsystem();
  CommandXboxController m_driver = new CommandXboxController(OIConstants.kDriverControllerPort);

  // A chooser for autonomous commands
  private final SendableChooser<Command> m_autoChooser;
    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter m_RotLimiter = new SlewRateLimiter(5);

  private final String controllerMode = "x";

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

  // Configure default commands
  if (controllerMode == "x") {
      m_robotDrive.setDefaultCommand(
          // The left stick controls translation of the robot.
          // Turning is controlled by the X axis of the right stick.
          new RunCommand(
              () ->
                  m_robotDrive.drive(
                      m_xSpeedLimiter.calculate( -m_driver.getLeftY() ) * SwerveConstants.kMaxSpeedTeleop,
                      m_ySpeedLimiter.calculate( -m_driver.getLeftX() ) * SwerveConstants.kMaxSpeedTeleop,
                      m_RotLimiter.calculate( -m_driver.getRightX() ) * ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND,
                      true),
              m_robotDrive));
  }
    m_autoChooser = AutoBuilder.buildAutoChooser();  // Default auto will be 'Commands.none()'

    SmartDashboard.putData("Auto Mode", m_autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  
  private void configureButtonBindings() {
    m_driver.leftBumper().whileTrue(Commands.runOnce(() -> m_index.runIndex(), m_index));
  }

 public Command getAutonomousCommand() {return m_autoChooser.getSelected();}
}