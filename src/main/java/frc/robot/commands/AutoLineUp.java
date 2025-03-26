// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.mechanisms.Elevator;
// import frc.robot.subsystems.mechanisms.ScoringMechanismPivot;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class AutoLineUp extends ParallelCommandGroup {
//   /** Creates a new AutoLineUp. */
//   private final Elevator m_elevator;
//   private final DriveSubsystem m_driveSubsytem;
//   private final ScoringMechanismPivot m_scoringMechPivot;
//   public AutoLineUp(Elevator ele, DriveSubsystem dri, ScoringMechanismPivot scp) {
//     // Add your commands in the addCommands() call, e.g.
//     m_elevator = ele;
//     addRequirements(m_elevator);
//     m_driveSubsytem = dri;
//     m_scoringMechPivot = scp;
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(new RunElevator(m_elevator)
//     .alongWith(Commands.runOnce(() -> m_driveSubsytem.isAutoYSpeed = true))
//     .alongWith(Commands.runOnce(() -> m_scoringMechPivot.rotatePivot(m_scoringMechPivot.scoringMechGoalAngle))
//     .onlyWhile((() -> m_elevator.getMotorPosition() >= ((327 * m_elevator.percentOfElevator) * .75)))));
//   }
// }