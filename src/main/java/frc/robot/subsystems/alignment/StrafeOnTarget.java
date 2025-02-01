// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.alignment;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.ReefAreas;
import frc.robot.Constants.ConstantsOffboard;

public class StrafeOnTarget {
  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
  private Pose2d targetPose = aprilTagFieldLayout.getTagPose(7).get().toPose2d();
  private PIDController m_turnCtrl = new PIDController(0.014, 0.015, 0.0013);
  private double heading;
  private double m_output;
  private boolean inZone = false;

public StrafeOnTarget() {}

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

    m_turnCtrl.reset();

    // Turns to target april tag if on the right allinace 
    return ReefAreas.getAngle(estimatedPose2d, alliance.get())
      .map(angle -> processRotation(angle, heading))
      .orElse(0.0);
  }

  private double processRotation(double angle, double heading) {
    if (Math.abs(m_turnCtrl.getError()) <= m_turnCtrl.getErrorTolerance()) {
      return 0;
    }
    
    m_turnCtrl.setSetpoint(angle);
    m_output = MathUtil.clamp(m_turnCtrl.calculate(heading), -1.0, 1.0);

    return m_output * ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND;
  }
}