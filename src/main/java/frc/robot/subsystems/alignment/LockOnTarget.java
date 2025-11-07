// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.alignment;

import java.util.Vector;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConstantsOffboard;

public class LockOnTarget {
  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();
  private Pose2d targetPose = aprilTagFieldLayout.getTagPose(7).get().toPose2d();
  private PIDController m_turnCtrl = new PIDController(0.014, 0.015, 0.0013);
  private double goalAngle;
  private double heading;
  private double m_output;
  private double wheelCirc = 0.319185814;
  private double shooterVelocity;
  private Vector<Double> ShooterVector;
  private double CompFactor = 3.0;

public LockOnTarget() {
    // SmartDashboard.putNumber("P", 0.02);
    // SmartDashboard.putNumber("I", 0.0003);
    // SmartDashboard.putNumber("D", 0.0015);
}

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
  public double execute(Pose2d estimatedPose2d, Vector<Double> robotVector) {
    // SmartDashboard.putNumber("Tolerance", m_turnCtrl.getErrorTolerance());
    // m_turnCtrl.setTolerance(2.00);
    if (3.0 > estimatedPose2d.getX() && estimatedPose2d.getX() > 5.0 && estimatedPose2d.getY() > 3 && estimatedPose2d.getY() < 5.0) {}
    var alliance = DriverStation.getAlliance();
    if (alliance.get() == DriverStation.Alliance.Red) {
      targetPose = aprilTagFieldLayout.getTagPose(4).get().toPose2d();
    }
    else {
      targetPose = aprilTagFieldLayout.getTagPose(7).get().toPose2d();
    }
    heading = estimatedPose2d.getRotation().getDegrees();

    // shooterVelocity = SmartDashboard.getNumber("Flywheel top RPM", 0.0);

    if (shooterVelocity <= 10) {
      shooterVelocity = 0;
    }

    double distanceToTargetX = estimatedPose2d.getX() - targetPose.getX();
    double distanceToTargetY = estimatedPose2d.getY() - targetPose.getY();
    // double distanceToTargetHyp = Math.abs(Math.sqrt(Math.pow(distanceToTargetX, 2) + Math.pow(distanceToTargetY, 2)));

    if (alliance.get() == DriverStation.Alliance.Red) {
      goalAngle = (Math.toDegrees(Math.atan(distanceToTargetY / distanceToTargetX)) - 11 + (2 * (16.459 - estimatedPose2d.getX()) / 3)) - 180;
    }
    else {
      goalAngle = (Math.toDegrees(Math.atan(distanceToTargetY / distanceToTargetX)) - 11 + (2 * estimatedPose2d.getX() / 3));
    }
    if (goalAngle > 180) goalAngle -= 360;
    if (goalAngle < -180) goalAngle += 360;
    // SmartDashboard.putNumber("Goal Angle", goalAngle);

    // goalAngle += Math.toDegrees(Math.atan2(ShooterVector.get(0), ShooterVector.get(1)) - Math.atan2(robotVector.get(0), robotVector.get(1)));

    double v_rx = robotVector.get(0); // m/s, robot velocity in x direction
    double v_ry = robotVector.get(1) * CompFactor; // m/s, robot velocity in y direction

    if (robotVector.get(1) >= 0) {
      v_ry = robotVector.get(1) * CompFactor; // m/s, robot velocity in y direction
    }
    else {
      v_ry = robotVector.get(1) * CompFactor * -1; // m/s, robot velocity in y direction
    }
    // double n = SmartDashboard.getNumber("Vision velocity", 0.0) * 60;   // rotations per second
    // double C = 0.319185814;    // meters, circumference of the wheel
    // double theta_s = goalAngle; // degrees, current facing angle of the shooter

    // // Calculate projectile speed
    // // double v_p = n * C;

    // // Convert theta_s to radians for calculation
    // double theta_s_rad = Math.toRadians(theta_s);

    // // Calculate projectile's initial velocity components
    // double v_px = v_p * Math.cos(theta_s_rad);
    // double v_py = v_p * Math.sin(theta_s_rad);

    // // Calculate effective projectile velocity components
    // double v_px_eff = v_px + v_rx;
    // double v_py_eff = v_py + v_ry;

    // // Calculate the effective shooting angle
    // double theta_eff = Math.toDegrees(Math.atan2(v_py_eff, v_px_eff));

    // m_turnCtrl.reset();
    // if (n >= 100) {    
    //   m_turnCtrl.setSetpoint(theta_eff);
    // }
    // else {
    //   m_turnCtrl.setSetpoint(goalAngle);
    // }
    // SmartDashboard.putNumber("Tolerance", m_turnCtrl.getErrorTolerance());
    // SmartDashboard.putNumber("Error", m_turnCtrl.getError());
    // m_output = MathUtil.clamp(m_turnCtrl.calculate(heading), -1.0, 1.0);

    // m_output = MathUtil.clamp(heading, -1.0, 1.0);

    // SmartDashboard.putNumber("m_output", m_output);
    if (Math.abs(m_turnCtrl.getError()) <= m_turnCtrl.getErrorTolerance()) {
      return 0;
    }
    else {
      return m_output * ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND;
    }
    // m_drive.drive(0.0, 0.0, m_output * ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND, true);
  }
}