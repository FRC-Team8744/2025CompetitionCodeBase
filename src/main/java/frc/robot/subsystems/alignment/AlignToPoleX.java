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
import frc.robot.DriveModifier;
import frc.robot.RotationEnum;
import frc.robot.Constants.SwerveConstants;
import frc.robot.isInAreaEnum;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToPoleX extends DriveModifier {
  /** Creates a new AlignToPole. */
  private PIDController m_driveCtrl = new PIDController(0.3, 0, 0);
  // private double heading;
  private double m_output;
  public boolean hasReachedX = false;  
  public double xOffset;
  public double robotX;
  public AlignToPoleX() {
    super(true, false, false);
  }

  @Override
  protected void initialize() {
    // m_driveCtrl.enableContinuousInput(-180, 180);
    m_driveCtrl.setTolerance(0.1);
    m_driveCtrl.reset();
  }

  public double execute(Pose2d estimatedPose2d) {
    double[] translatedRobotPosition = calculateTransformation(new double[]{estimatedPose2d.getX(), estimatedPose2d.getY()}, isInAreaEnum.areaEnum.getAngle() * -1);

    robotX = translatedRobotPosition[0];
    double goalX;
    if (Constants.scoringMode == "Coral") {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        if (DriverStation.isAutonomous()) {
          goalX = 3.1723; // 3.17
        } else {
          goalX = 3.1746;
        }
      }
      else {
        if (DriverStation.isAutonomous()) {
          goalX = 11.7808;
        } else {
          goalX = 11.7762;
        }
      }
    } else if (Constants.scoringMode == "Algae") {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        goalX = 3.0746;
      } else {
        goalX = 11.8143;
      }
    } else {
      goalX = 3;
    }

    // SmartDashboard.putNumber("Goal X", goalX);
    SmartDashboard.putNumber("Robot X", robotX);

    xOffset = goalX - robotX;

    // if (Math.abs(xOffset) >= 3.0) {
    //   xOffset = 0;
    // }

    // SmartDashboard.putNumber("X Offset", xOffset);
    // SmartDashboard.putNumber("Goal X", goalX);

    m_driveCtrl.setSetpoint(xOffset);

    m_output = MathUtil.clamp(m_driveCtrl.calculate(0), -1.0, 1.0);

    if (Math.abs(m_driveCtrl.getError()) <= m_driveCtrl.getErrorTolerance()) {
      hasReachedX = true;
      return 0;
    }
    else {
      hasReachedX = false;
      return m_output * SwerveConstants.kMaxSpeedTeleop;
    }
  }

  public double[] calculateTransformation(double[] positionToRotate, double angle) {
    var alliance = DriverStation.getAlliance();
    double[] translationAmount;
    if (alliance.get() == DriverStation.Alliance.Blue) {
      // translationAmount = new double[]{4.489337, 4.05128984}; // 4.026
      translationAmount = new double[]{4.49, 4.03};
    }
    else {
      // translationAmount = new double[]{12.8945, 4.05128984}; // 12.8945
      translationAmount = new double[]{13.06, 4.03};
    }

    double X = ((positionToRotate[0] - translationAmount[0]) * Math.cos(Math.toRadians(angle))) 
    + ((positionToRotate[1] - translationAmount[1]) * -Math.sin(Math.toRadians(angle)));
    double Y = ((positionToRotate[0] - translationAmount[0]) * Math.sin(Math.toRadians(angle))) 
    + ((positionToRotate[1] - translationAmount[1]) * Math.cos(Math.toRadians(angle)));

    X += translationAmount[0];
    Y += translationAmount[1];

    return new double[]{X, Y};
  }

  @Override
  public boolean shouldRun(DriveSubsystem drive) {
    return Constants.isAutoXSpeed && Constants.isAutoRotate == RotationEnum.STRAFEONTARGET;
  }

  @Override
  protected void doExecute(DriveSubsystem drive) {
    Constants.autoXSpeed = execute(drive.getEstimatedPose());
  }
}