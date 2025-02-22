// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.alignment;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.isInAreaEnum;

public class AlignToPole {
  /** Creates a new AlignToPole. */
  private PIDController m_driveCtrl = new PIDController(0.2, 0, 0);
  // private double heading;
  private double m_output;
  public AlignToPole() {}

  public void initialize() {
    // m_driveCtrl.enableContinuousInput(-180, 180);
    m_driveCtrl.setTolerance(0.02);
    m_driveCtrl.reset();
  }

  public double execute(boolean rightPoint, Pose2d estimatedPose2d) {
    double[] translatedRobotPosition = calculateTransformation(new double[]{estimatedPose2d.getX(), estimatedPose2d.getY()}, isInAreaEnum.areaEnum.getAngle() * -1);

    double robotY = translatedRobotPosition[1];
    double goalY = rightPoint ? Constants.rightPoint[1] : Constants.leftPoint[1];

    SmartDashboard.putNumber("Goal Y", goalY);
    SmartDashboard.putNumber("Robot Y", robotY);

    double yOffset = goalY - robotY;

    SmartDashboard.putNumber("Y Offset", yOffset);

    m_driveCtrl.setSetpoint(yOffset);

    m_output = MathUtil.clamp(m_driveCtrl.calculate(0), -1.0, 1.0);

    if (Math.abs(m_driveCtrl.getError()) <= m_driveCtrl.getErrorTolerance()) {
      return 0;
    }
    else {
      return m_output * SwerveConstants.kMaxSpeedTeleop;
    }
  }

  public double[] calculateTransformation(double[] positionToRotate, double angle) {
    var alliance = DriverStation.getAlliance();
    double[] translationAmount;
    if (alliance.get() == DriverStation.Alliance.Blue) {
      translationAmount = new double[]{4.489337, 4.05128984}; // 4.026
    }
    else {
      translationAmount = new double[]{12.8945, 4.02588984}; // 4.026
    }

    double X = ((positionToRotate[0] - translationAmount[0]) * Math.cos(Math.toRadians(angle))) 
    + ((positionToRotate[1] - translationAmount[1]) * -Math.sin(Math.toRadians(angle)));
    double Y = ((positionToRotate[0] - translationAmount[0]) * Math.sin(Math.toRadians(angle))) 
    + ((positionToRotate[1] - translationAmount[1]) * Math.cos(Math.toRadians(angle)));

    X += translationAmount[0];
    Y += translationAmount[1];

    return new double[]{X, Y};
  }
}
