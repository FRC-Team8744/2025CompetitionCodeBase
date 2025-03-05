// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ScoringMechSensor;
import frc.robot.subsystems.alignment.AlignToClimb;
import frc.robot.subsystems.alignment.AlignToPole;
import frc.robot.subsystems.alignment.LockOnTarget;
// import frc.robot.subsystems.mechanisms.AlgaeMechanism;
import frc.robot.subsystems.mechanisms.Climber;
import frc.robot.subsystems.mechanisms.CoralScoring;
import frc.robot.subsystems.mechanisms.Elevator;
import frc.robot.subsystems.mechanisms.Intake;
import frc.robot.subsystems.mechanisms.IntakePivot;
import frc.robot.subsystems.mechanisms.ScoringMechanismPivot;
import frc.robot.subsystems.vision.PhotonVisionGS;
import frc.robot.subsystems.vision.PhotonVisionGS2;
// import frc.robot.commands.AutoLineUp;
import frc.robot.commands.RunElevator;
// import frc.robot.commands.DropCoral;
import frc.robot.commands.RunIntake;
import frc.robot.commands.TeleopScore;

/** Add your docs here. */
public class AutoCommandManager {
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    public HolonomicDriveController holonomicDriveController;
    
    public static boolean isSim;

    public TrajectoryConfig forwardConfig;
    public TrajectoryConfig reverseConfig;

    public AutoCommandManager(
        Elevator m_elevator,
        Intake m_intake,
        // AlgaeMechanism m_algaMechanism,
        Climber m_climber,
        CoralScoring m_coralScoring,
        IntakePivot m_intakePivot,
        ScoringMechanismPivot  m_scoringMechPivot,
        PhotonVisionGS m_visionGS,
        PhotonVisionGS2 m_visionGS2,
        AlignToPole m_alignToPole,
        LockOnTarget m_lockOnTarget,
        DriveSubsystem m_robotDrive,
        AlignToClimb m_alignToClimb,
        ScoringMechSensor m_scoringMechSensor)
         {

        configureNamedCommands(
            m_elevator,
            m_intake,
            // m_algaMechanism,
            m_climber,
            m_coralScoring,
            m_intakePivot,
            m_scoringMechPivot,
            m_visionGS,
            m_visionGS2,
            m_alignToPole,
            m_lockOnTarget,
            m_robotDrive,
            m_alignToClimb,
            m_scoringMechSensor
      );

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0,
            new TrapezoidProfile.Constraints(AutoConstants.kMaxAngularSpeedRadiansPerSecond, 
                AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        holonomicDriveController = new HolonomicDriveController(
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController);

        forwardConfig = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond, 
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(SwerveConstants.kDriveKinematics);

        reverseConfig = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond, 
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(SwerveConstants.kDriveKinematics)
            .setReversed(true);

        isSim = true;

        PathPlannerAuto m_test = new PathPlannerAuto("Height Matters");

        m_chooser.setDefaultOption("None", new InstantCommand());

        m_chooser.addOption("Height Matters", m_test);

        SmartDashboard.putData(m_chooser);
    }

    public SendableChooser<Command> getChooser() {
        return m_chooser;
    }

    public Command getAutoManagerSelected() {
        return m_chooser.getSelected();
    }

    public SwerveControllerCommand trajectoryCommand(Trajectory trajectory, DriveSubsystem m_robotDrive) {
        return new SwerveControllerCommand(
            trajectory,
            m_robotDrive::getPose,
            SwerveConstants.kDriveKinematics,
            holonomicDriveController,
            m_robotDrive::setModuleStates,
            m_robotDrive);
    }

    public void configureNamedCommands(

        Elevator m_elevator,
        Intake m_intake,
        // AlgaeMechanism m_algaMechanism,
        Climber m_climber,
        CoralScoring m_coralScoring,
        IntakePivot m_intakePivot,
        ScoringMechanismPivot  m_scoringMechPivot,
        PhotonVisionGS m_visionGS,
        PhotonVisionGS2 m_visionGS2,
        AlignToPole m_alignToPole,
        LockOnTarget m_lockOnTarget,
        DriveSubsystem m_robotDrive,
        AlignToClimb m_alignToClimb,
        ScoringMechSensor m_scoringMechSensor
        
        ) { 
            
        // NamedCommands.registerCommand("Auto line up", new AutoLineUp(m_elevator, m_robotDrive, m_scoringMechPivot));
        NamedCommands.registerCommand("Auto rotate", Commands.runOnce(() -> m_robotDrive.isAutoRotate = m_robotDrive.isAutoRotate == RotationEnum.STRAFEONTARGET ? RotationEnum.NONE : RotationEnum.STRAFEONTARGET));
        NamedCommands.registerCommand("Run Intake", new RunIntake(m_intake, m_intakePivot, m_coralScoring, m_scoringMechSensor));
        NamedCommands.registerCommand("Drop Coral", new TeleopScore(m_coralScoring).finallyDo((() -> {m_robotDrive.isAutoYSpeed = false; m_robotDrive.isAutoRotate = m_robotDrive.isAutoRotate == RotationEnum.STRAFEONTARGET ? RotationEnum.NONE : RotationEnum.STRAFEONTARGET;})));        
    }
}