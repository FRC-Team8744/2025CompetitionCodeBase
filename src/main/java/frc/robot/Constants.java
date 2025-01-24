// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.lang.reflect.Array;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final int kDebugLevel = 0; // 0 = None, 1 = Errors, 2 = Info, 3 = Debug and USB data log
  
  public static final int kMaxSpeedPercentAuto = 100;
  public static final int kMaxSpeedPercentTeleop = 20;
  public static final int kMaxAccelerationPercent = 100;
  public static final double kDriverSpeedLimit = 1; // sets how much the max speed is modified by when you press down on the left stick basicly make go slower the default is 1 btw 

  public static final TalonFXConfiguration cfg = new TalonFXConfiguration();
  public static final Slot0Configs cfg0 = new Slot0Configs();

  // 17.55 is the distance of the field in meters
  // This gets the points of the triangles to calc if it can strafe 
  public static final Pose2d[] redBorder6 = new Pose2d[]{new Pose2d(17.55 - 4.319, 3.67, new Rotation2d(0)), new Pose2d(17.55 - -0.191, 0.67, new Rotation2d(0)), new Pose2d(17.55 - 4.319, -0.33, new Rotation2d(0))};
  public static final Pose2d[] redBorder7 = new Pose2d[]{new Pose2d(17.55 - 4.12, 4.0, new Rotation2d(0)), new Pose2d(17.55 - -0.38, 7.0, new Rotation2d(0)), new Pose2d(17.55 - -0.38, 1.0, new Rotation2d(0))};
  public static final Pose2d[] redBorder8 = new Pose2d[]{new Pose2d(17.55 - 4.319, 4.33, new Rotation2d(0)), new Pose2d(17.55 - 4.319, 8.33, new Rotation2d(0)), new Pose2d(17.55 - -0.191, 7.33, new Rotation2d(0))};
  public static final Pose2d[] redBorder9 = new Pose2d[]{new Pose2d(17.55 - 4.691, 4.33, new Rotation2d(0)), new Pose2d(17.55 - 10.191, 7.33, new Rotation2d(0)), new Pose2d(17.55 - 4.691, 8.33, new Rotation2d(0))};
  public static final Pose2d[] redBorder10 = new Pose2d[]{new Pose2d(17.55 - 4.88, 4.0, new Rotation2d(0)), new Pose2d(17.55 - 10.38, 1.0, new Rotation2d(0)), new Pose2d(17.55 - 10.38, 7.0, new Rotation2d(0))};
  public static final Pose2d[] redBorder11 = new Pose2d[]{new Pose2d(17.55 - 4.691, 3.67, new Rotation2d(0)), new Pose2d(17.55 - 4.691, -0.33, new Rotation2d(0)), new Pose2d(17.55 - 10.191, 0.67, new Rotation2d(0))};
  public static final Pose2d[] blueBorder17 = new Pose2d[]{new Pose2d(4.319, 3.67, new Rotation2d(0)), new Pose2d(4.319, -0.33, new Rotation2d(0)), new Pose2d(-0.191, 0.67, new Rotation2d(0))};
  public static final Pose2d[] blueBorder18 = new Pose2d[]{new Pose2d(4.12, 4.0, new Rotation2d(0)), new Pose2d(-0.38, 1.0, new Rotation2d(0)), new Pose2d(-0.38, 7.0, new Rotation2d(0))};
  public static final Pose2d[] blueBorder19 = new Pose2d[]{new Pose2d(4.319, 4.33, new Rotation2d(0)), new Pose2d(-0.191, 7.33, new Rotation2d(0)), new Pose2d(4.319, 8.33, new Rotation2d(0))};
  public static final Pose2d[] blueBorder20 = new Pose2d[]{new Pose2d(4.691, 4.33, new Rotation2d(0)), new Pose2d(4.691, 8.33, new Rotation2d(0)), new Pose2d(10.191, 7.33, new Rotation2d(0))};
  public static final Pose2d[] blueBorder21 = new Pose2d[]{new Pose2d(4.88, 4.0, new Rotation2d(0)), new Pose2d(10.38, 7.0, new Rotation2d(0)), new Pose2d(10.38, 1.0, new Rotation2d(0))};
  public static final Pose2d[] blueBorder22 = new Pose2d[]{new Pose2d(4.691, 3.67, new Rotation2d(0)), new Pose2d(10.191, 0.67, new Rotation2d(0)), new Pose2d(4.691, -0.33, new Rotation2d(0))};

// public static final Pose2d[] 

  public Constants() {
    cfg.Voltage.PeakForwardVoltage = 12;
    cfg.Voltage.PeakReverseVoltage = -12;
    cfg.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    cfg.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = 40.0;
    cfg0.kV = Constants.ConstantsOffboard.KRAKEN_V;
    cfg0.kP = Constants.ConstantsOffboard.KRAKEN_P;
    cfg0.kI = Constants.ConstantsOffboard.KRAKEN_I;
    cfg0.kD = Constants.ConstantsOffboard.KRAKEN_D;
    cfg.withSlot0(cfg0);
  }

  public static final class MechanismConstants {}

  public static final class SwerveConstants {
    public static final double kMaxSpeedMetersPerSecond = (5.94 * kMaxSpeedPercentAuto) / 100;
    public static final double kMaxSpeedTeleop = (10.0 * kMaxSpeedPercentTeleop) / 100;

    // The drive classes use the NWU axes convention (North-West-Up as external reference in the world frame).
    // The positive X axis points ahead, the positive Y axis points left, and the positive Z axis points up.
    // We use NWU here because the rest of the library, and math in general, use NWU axes convention.
    // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html#axis-conventions
    public static final int kFrontLeftDriveMotorPort = 24;
    public static final int kFrontRightDriveMotorPort = 23;
    public static final int kRearLeftDriveMotorPort = 22;
    public static final int kRearRightDriveMotorPort = 21;

    public static final int kFrontLeftTurningMotorPort = 3;
    public static final int kFrontRightTurningMotorPort = 1;
    public static final int kRearLeftTurningMotorPort = 7;
    public static final int kRearRightTurningMotorPort = 5;

    public static final int kFrontLeftMagEncoderPort = 10;
    public static final int kFrontRightMagEncoderPort = 9;
    public static final int kRearLeftMagEncoderPort = 11;
    public static final int kRearRightMagEncoderPort = 12;

    // Only disable the steering angle optimizer when measuring the CANcoder offsets!
    public static final boolean DISABLE_ANGLE_OPTIMIZER = false;

    // Note: Zeroing the CanCoder in Tuner X doesn't seem to affect the reported absolute position.
    public static final double kFrontLeftMagEncoderOffsetDegrees_NoNo = 0.685547 * 360; // 10
    public static final double kFrontRightMagEncoderOffsetDegrees_NoNo = 0.369629 * 360; // 9
    public static final double kRearLeftMagEncoderOffsetDegrees_NoNo = 0.030273 * 360; // 11
    public static final double kRearRightMagEncoderOffsetDegrees_NoNo = 0.723896 * 360; // 12

    public static final double kFrontLeftMagEncoderOffsetDegrees_Swivels = 81.12;
    public static final double kFrontRightMagEncoderOffsetDegrees_Swivels = 133.77;
    public static final double kRearLeftMagEncoderOffsetDegrees_Swivels = 11.25;
    public static final double kRearRightMagEncoderOffsetDegrees_Swivels = 66.71;

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(20.472);

    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(20.472);

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
          // This is the order all swerve module references need to be in!
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),  // Front Left Quadrant
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),  // Front Right Quadrant
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),  // Rear Left Quadrant
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));  // Rear Right Quadrant

    public static final int kIMU_ID = 13;

    public static int kSwerveFL_enum = 0;
    public static int kSwerveFR_enum = 1;
    public static int kSwerveRL_enum = 2;
    public static int kSwerveRR_enum = 3;
  }

  public static final class ConstantsOffboard {
    public static final int kMaximumSparkMaxRPM = 6000;
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final double DRIVE_GEAR_RATIO = 5.36 / 1.0; // 5.36:1
    public static final double DRIVE_ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
    public static final double DRIVE_RPM_TO_METERS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;
    public static final double ANGLE_GEAR_RATIO = (150 / 7) / 1.0; // 150/7:1
    public static final double ANGLE_ROTATIONS_TO_RADIANS = (Math.PI * 2) / ANGLE_GEAR_RATIO;
    public static final double ANGLE_RPM_TO_RADIANS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;

    /** Current limiting. */
    public static final int DRIVE_CURRENT_LIMIT = 40;
    public static final int ANGLE_CURRENT_LIMIT = 20;

    public static final boolean DRIVE_MOTOR_PROFILED_MODE = true;
    /** Angle motor PID values for speed/acceleration limited mode. */
    // Reference: https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java
    public static final double DRIVE_KP_PROFILED = 0.01;
    public static final double DRIVE_KI_PROFILED = 0.0;
    public static final double DRIVE_KD_PROFILED = 0.0;
    public static final double DRIVE_KF_PROFILED = 0.23;
    public static final double DRIVE_MAX_VEL_PROFILED = kMaximumSparkMaxRPM;  // Maximum Velocity, RPM
    public static final double DRIVE_MAX_ACC_PROFILED = 20000;  // Maximum Acceleration, RPM^2
    public static final double DRIVE_MAX_ERR_PROFILED = 0.02;  // Error tolerance of PID controller, rotations

    /** Drive motor PID values. */
    public static final double DRIVE_KP = 0.25;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KF = 0.25;

    public static final double KRAKEN_V = 0.32;
    public static final double KRAKEN_P = 0.11;
    public static final double KRAKEN_I = 0.48;
    public static final double KRAKEN_D = 0.01;

    public static final boolean ANGLE_MOTOR_PROFILED_MODE = false;
    /** Angle motor PID values for speed/acceleration limited mode. */
    // Reference: https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java
    public static final double ANGLE_KP_PROFILED = 0.00075;
    public static final double ANGLE_KI_PROFILED = 0.0;
    public static final double ANGLE_KD_PROFILED = 0.0;
    public static final double ANGLE_KF_PROFILED = 0.0003;
    public static final double ANGLE_MAX_VEL_PROFILED = kMaximumSparkMaxRPM;  // Maximum Velocity, RPM
    public static final double ANGLE_MAX_ACC_PROFILED = 20000;  // Maximum Acceleration, RPM^2
    public static final double ANGLE_MAX_ERR_PROFILED = 0.02;  // Error tolerance of PID controller, rotations

    /** Angle motor PID values. */
    public static final double ANGLE_KP = 1.5;
    public static final double ANGLE_KI = 0.0;
    public static final double ANGLE_KD = 0.1;
    public static final double ANGLE_KF = 0.0;
    
    /** Swerve constraints. */
    public static final double MAX_SPEED_IN_PERCENT = 100.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 0.1 * MAX_SPEED_IN_PERCENT;
    public static final double MAX_ANGULAR_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND * 4/3;
    public static final double MAX_ANGULAR_DEGREES_PER_SECOND = Math.toDegrees(MAX_ANGULAR_RADIANS_PER_SECOND);

    /** Inversions. */
    public static final boolean DRIVE_MOTOR_INVERSION = true;
    public static final boolean ANGLE_MOTOR_INVERSION = true;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = 0.3;
    public static final double kRotationDeadband = 1.8;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = (4.4 * kMaxSpeedPercentAuto) / 100;
    public static final double kMaxAccelerationMetersPerSecondSquared = (30 * kMaxAccelerationPercent) / 100;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
