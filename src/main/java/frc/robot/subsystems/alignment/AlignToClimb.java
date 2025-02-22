// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.alignment;

import java.util.Vector;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ConstantsOffboard;

public class AlignToClimb {
  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();
  private Pose2d targetPose = aprilTagFieldLayout.getTagPose(7).get().toPose2d();
  private PIDController m_turnCtrl = new PIDController(0.014, 0.015, 0.0013);
  private double goalAngle;
  private double heading;
  private double m_output;
  private boolean inZone = false;

public AlignToClimb() {}

  // Called when the command is initially scheduled.
  public void initialize() {
    // m_turnCtrl.setP(SmartDashboard.getNumber("P", 0));
    // m_turnCtrl.setI(SmartDashboard.getNumber("I", 0));
    // m_turnCtrl.setD(SmartDashboard.getNumber("D", 0));
    m_turnCtrl.enableContinuousInput(-180, 180);
    m_turnCtrl.setTolerance(2.00);
    m_turnCtrl.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  public double execute(Pose2d estimatedPose2d) {
    // if (3.0 > estimatedPose2d.getX() && estimatedPose2d.getX() > 5.0 && estimatedPose2d.getY() > 3 && estimatedPose2d.getY() < 5.0) {}
    var alliance = DriverStation.getAlliance();

    heading = estimatedPose2d.getRotation().getDegrees();

    SmartDashboard.putNumber("Goal Angle", goalAngle);

    m_turnCtrl.reset();

    // Turns to target april tag if on the right allinace 
    if (alliance.get() == DriverStation.Alliance.Blue) {
      m_turnCtrl.setSetpoint(180); // 0
    }
    else {
      m_turnCtrl.setSetpoint(0); // 180
    }

    m_output = MathUtil.clamp(m_turnCtrl.calculate(heading), -1.0, 1.0);
    
    if (Math.abs(m_turnCtrl.getError()) <= m_turnCtrl.getErrorTolerance()) {
      return 0;
    }
    else {
      return m_output * ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND;
    }
  }
}