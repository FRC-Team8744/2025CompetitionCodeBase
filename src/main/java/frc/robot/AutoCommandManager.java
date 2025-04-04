// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.util.Named;
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
import frc.robot.subsystems.mechanisms.AlgaeMechanism;
import frc.robot.subsystems.mechanisms.Climber;
import frc.robot.subsystems.mechanisms.CoralScoring;
import frc.robot.subsystems.mechanisms.Elevator;
import frc.robot.subsystems.mechanisms.Intake;
import frc.robot.subsystems.mechanisms.IntakePivot;
import frc.robot.subsystems.mechanisms.ScoringMechanismPivot;
import frc.robot.subsystems.vision.PhotonVisionGS;
import frc.robot.subsystems.vision.PhotonVisionGS2;
import frc.robot.commands.AutoScore;
import frc.robot.commands.ElevatorGoDownAuto;
import frc.robot.commands.ElevatorToScore;
import frc.robot.commands.ElevatorToScoreAuto;
import frc.robot.commands.NoTwoPieces;
import frc.robot.commands.NoTwoPiecesAuto;
// import frc.robot.commands.AutoLineUp;
import frc.robot.commands.RunElevator;
// import frc.robot.commands.DropCoral;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunIntakeAuto;
import frc.robot.commands.RunIntakeAutoKickout;
import frc.robot.commands.RunIntakeAutoSource;
import frc.robot.commands.TeleopScore;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.alignment.AlignToPoleX;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.ColorInterface;



/** Add your docs here. */
public class AutoCommandManager {
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    public HolonomicDriveController holonomicDriveController;
    
    public static boolean isSim;

    public TrajectoryConfig forwardConfig;
    public TrajectoryConfig reverseConfig;

    public AutoCommandManager(
        LEDS m_leds, 
        Elevator m_elevator,
        Intake m_intake,
        AlgaeMechanism m_algaMechanism,
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
            m_algaMechanism,
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
            m_scoringMechSensor, 
            m_leds
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

        PathPlannerAuto m_test = new PathPlannerAuto("1 Piece Test");
        PathPlannerAuto m_intaketest = new PathPlannerAuto("Coral Intaking Auto");
        PathPlannerAuto m_4plrs = new PathPlannerAuto("4plrs");
        PathPlannerAuto m_4pgilrs = new PathPlannerAuto("4pgilrs");
        PathPlannerAuto m_4pfilrs = new PathPlannerAuto("4pfilrs");
        PathPlannerAuto m_4psilrs = new PathPlannerAuto("4psilrs");

        m_chooser.setDefaultOption("None", new InstantCommand());

        // m_chooser.addOption("1 Piece Test", m_test);
        // m_chooser.addOption("Intake Test", m_intaketest);
        m_chooser.addOption("4plrs", m_4plrs);
        m_chooser.addOption("4pgilrs", m_4pgilrs);
        m_chooser.addOption("4pfilrs", m_4pfilrs);
        m_chooser.addOption("4psilrs", m_4psilrs);

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
        AlgaeMechanism m_algaMechanism,
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
        ScoringMechSensor m_scoringMechSensor,
        LEDS m_leds
        
        ) { 
            
        NamedCommands.registerCommand("AutoLineUp", Commands.runOnce(() -> m_robotDrive.isAutoRotate = RotationEnum.STRAFEONTARGET));
        NamedCommands.registerCommand("L1", Commands.runOnce(() -> m_elevator.setScoringPreset(.25, -60, "L1", .25, -60, "Processor")));
        NamedCommands.registerCommand("L2", Commands.runOnce(() -> m_elevator.setScoringPreset(.33, -60, "L2", .33, -60, "L2")));
        NamedCommands.registerCommand("L3", Commands.runOnce(() -> m_elevator.setScoringPreset(.53, -60, "L3", .53, -60, "L3")));
        NamedCommands.registerCommand("L4", Commands.runOnce(() -> m_elevator.setScoringPreset(.9, -210, "L4", .9, -200, "Net")));
        NamedCommands.registerCommand("LeftPole", Commands.runOnce(() -> m_robotDrive.leftPoint = true));
        NamedCommands.registerCommand("RightPole", Commands.runOnce(() -> m_robotDrive.leftPoint = false));
        // NamedCommands.registerCommand("Auto rotate", Commands.runOnce(() -> m_robotDrive.isAutoRotate = m_robotDrive.isAutoRotate == RotationEnum.STRAFEONTARGET ? RotationEnum.NONE : RotationEnum.STRAFEONTARGET));
        NamedCommands.registerCommand("RunIntake", new RunIntakeAuto(m_intake, m_intakePivot, m_coralScoring, m_scoringMechSensor, m_algaMechanism, new ElevatorToScoreAuto(m_elevator, m_robotDrive, m_scoringMechPivot, m_scoringMechSensor), new NoTwoPieces(m_intake, m_intakePivot)));
        NamedCommands.registerCommand("NoTwoPieces", new NoTwoPiecesAuto(m_intake, m_intakePivot, m_scoringMechSensor));
        NamedCommands.registerCommand("ElevatorToScore", new ElevatorToScoreAuto(m_elevator, m_robotDrive, m_scoringMechPivot, m_scoringMechSensor));
        NamedCommands.registerCommand("ElevatorDown", new ElevatorGoDownAuto(m_elevator, m_scoringMechPivot));
        NamedCommands.registerCommand("ScoreCoral", new AutoScore(m_coralScoring, m_elevator, m_intake, m_intakePivot, m_scoringMechSensor).finallyDo((() -> {m_robotDrive.isAutoYSpeed = false; m_robotDrive.isAutoRotate = m_robotDrive.isAutoRotate == RotationEnum.STRAFEONTARGET ? RotationEnum.NONE : RotationEnum.STRAFEONTARGET;})));  
        NamedCommands.registerCommand("LEDS", Commands.runOnce(() -> m_leds.SetSegmentByVision(m_robotDrive.m_alignToPoleX.hasReachedX, m_robotDrive.m_alignToPole.hasReachedY, m_robotDrive.isAutoYSpeed, m_robotDrive.isAutoXSpeed, Color.kRed, ColorInterface.L1, ColorInterface.L1, Color.kBlue, 50)));      
        NamedCommands.registerCommand("RunIntakeAutoKickout", new RunIntakeAutoKickout(m_intake, m_intakePivot, m_coralScoring, m_scoringMechSensor, m_algaMechanism, new ElevatorToScoreAuto(m_elevator, m_robotDrive, m_scoringMechPivot, m_scoringMechSensor), new NoTwoPieces(m_intake, m_intakePivot)));
        NamedCommands.registerCommand("IntakeUp", Commands.runOnce(() -> m_intakePivot.intakeDown(-350)));
        NamedCommands.registerCommand("SourceIntake", new RunIntakeAutoSource(m_intake, m_intakePivot, m_coralScoring, m_scoringMechSensor, m_algaMechanism, null, null));
    }
}