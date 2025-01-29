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
import frc.robot.Constants.ConstantsOffboard;

public class StrafeOnTarget {
  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
  private Pose2d targetPose = aprilTagFieldLayout.getTagPose(7).get().toPose2d();
  private PIDController m_turnCtrl = new PIDController(0.014, 0.015, 0.0013);
  private double goalAngle;
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

    if (goalAngle > 180) goalAngle -= 360;
    if (goalAngle < -180) goalAngle += 360;
    SmartDashboard.putNumber("Goal Angle", goalAngle);

    m_turnCtrl.reset();

    // Turns to target april tag if on the right allinace 
    if (alliance.get() == DriverStation.Alliance.Blue) {
      if (isInArea (Constants.blueBorder17, estimatedPose2d)) {
        m_turnCtrl.setSetpoint(60);
        inZone = true;
      }
      else if (isInArea (Constants.blueBorder18, estimatedPose2d)) {
        m_turnCtrl.setSetpoint(0);
        inZone = true;
      }
      else if (isInArea (Constants.blueBorder19, estimatedPose2d)) {
        m_turnCtrl.setSetpoint(300);
        inZone = true;
      }
      else if (isInArea (Constants.blueBorder20, estimatedPose2d)) {
        m_turnCtrl.setSetpoint(240);
        inZone = true;
      }
      else if (isInArea (Constants.blueBorder21, estimatedPose2d)) {
        m_turnCtrl.setSetpoint(180);
        inZone = true;
      }
      else if (isInArea (Constants.blueBorder22, estimatedPose2d)) {
        m_turnCtrl.setSetpoint(120);
        inZone = true;
      }
      else {
        inZone = false;
      }
    }
    else if  (alliance.get() == DriverStation.Alliance.Red) {
      if (isInArea (Constants.redBorder11, estimatedPose2d)) {
        m_turnCtrl.setSetpoint(60);
        inZone = true;
      }
      else if (isInArea (Constants.redBorder10, estimatedPose2d)) {
        m_turnCtrl.setSetpoint(0);
        inZone = true;
      }
      else if (isInArea (Constants.redBorder9, estimatedPose2d)) {
        m_turnCtrl.setSetpoint(300);
        inZone = true;
      }
      else if (isInArea (Constants.redBorder8, estimatedPose2d)) {
        m_turnCtrl.setSetpoint(240);
        inZone = true;
      }
      else if (isInArea (Constants.redBorder7, estimatedPose2d)) {
        m_turnCtrl.setSetpoint(180);
        inZone = true;
      }
      else if (isInArea (Constants.redBorder6, estimatedPose2d)) {
        m_turnCtrl.setSetpoint(120);
        inZone = true;
      }
      else {
        inZone = false;
      }
    }
    else {
      inZone = false;
    }

    m_output = MathUtil.clamp(m_turnCtrl.calculate(heading), -1.0, 1.0);

    // SmartDashboard.putBoolean("Is in area 17", isInArea(Constants.blueBorder17, estimatedPose2d));
    // SmartDashboard.putBoolean("Is in area 18", isInArea(Constants.blueBorder18, estimatedPose2d));
    // SmartDashboard.putBoolean("Is in area 19", isInArea(Constants.blueBorder19, estimatedPose2d));
    // SmartDashboard.putBoolean("Is in area 20", isInArea(Constants.blueBorder20, estimatedPose2d));
    // SmartDashboard.putBoolean("Is in area 21", isInArea(Constants.blueBorder21, estimatedPose2d));
    // SmartDashboard.putBoolean("Is in area 22", isInArea(Constants.blueBorder22, estimatedPose2d));

    if (inZone) {
      if (Math.abs(m_turnCtrl.getError()) <= m_turnCtrl.getErrorTolerance()) {
        return 0;
      }
      else {
        return m_output * ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND;
      }
    }
    else {
      return 0;
    }
  }

  /**
   * Calculates if you are within a certain amount of points
   * @param border Array of points to calculate if you are inside of
   * @param estimatedPose2d Current robot position
   * @return Returns if you are in the given area
   */
  public boolean isInArea(Pose2d[] border, Pose2d estimatedPose2d) {
    double degree = 0.0;
    for (int i = 0 ; i < border.length; i++) {
      Pose2d a = border[i];
      Pose2d b = border[(i + 1) % (border.length)];
      double x1 = a.getX();
      double x2 = b.getX();
      double y1 = a.getY();
      double y2 = b.getY();
      double targetX = estimatedPose2d.getX();
      double targetY = estimatedPose2d.getY();

      // Calculate distance of vector
      double A = Math.sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));
      double B = Math.sqrt((y1 - targetY) * (y1 - targetY) + (x1 - targetX) * (x1 - targetX));
      double C = Math.sqrt((y2 - targetY) * (y2 - targetY) + (x2 - targetX) * (x2 - targetX));

      // Calculate direction of vector
      double ta_x = x1 - targetX;
      double ta_y = y1 - targetY;
      double tb_x = x2 - targetX;
      double tb_y = y2 - targetY;

      double cross = tb_y * ta_x - tb_x * ta_y;
      boolean clockwise = cross < 0;

      // Calculate sum of angles
      if (clockwise) {
        degree = degree + Math.toDegrees(Math.acos((B * B + C * C - A * A) / (2.0 * B * C)));
      }
      else {
        degree = degree - Math.toDegrees(Math.acos((B * B + C * C - A * A) / (2.0 * B * C)));
      }
    }
    return Math.abs(Math.round(degree) - 360) <= 3;
  }
}