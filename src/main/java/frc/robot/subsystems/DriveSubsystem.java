// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.Vector;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AutoCommandManager;
import frc.robot.Constants;
import frc.robot.Constants.ConstantsOffboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.alignment.AlignToClimb;
import frc.robot.subsystems.alignment.StrafeOnTarget;
import frc.robot.subsystems.vision.PhotonVisionGS;
import frc.robot.subsystems.vision.PhotonVisionGS2;
import frc.robot.RotationEnum;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveSubsystem extends SubsystemBase {
  StructPublisher<Pose2d> pose_publisher = NetworkTableInstance.getDefault().getStructTopic("RobotPose", Pose2d.struct).publish();
  StructArrayPublisher<SwerveModuleState> swerve_publisher = NetworkTableInstance.getDefault().getStructArrayTopic("Swerve States", SwerveModuleState.struct).publish();

  double offset_FL = 0;
  double offset_RL = 0;
  double offset_FR = 0;
  double offset_RR = 0;
  
  private double m_DriverSpeedScale = 1.0;

  // Robot swerve modules
  private final SwerveModuleOffboard m_frontLeft;
  private final SwerveModuleOffboard m_rearLeft;
  private final SwerveModuleOffboard m_frontRight;
  private final SwerveModuleOffboard m_rearRight;

  private double autoRotateSpeed = 0;

  private double goalAngle = 0;
  private PIDController m_turnCtrl = new PIDController(0.03, 0.0065, 0.0035);
  private boolean roboNoSpino = true;
  private Timer rotationTimer = new Timer();

  final Timer m_timerX = new Timer();
  final Timer m_timerY = new Timer();

  public double xVelocity = 0;
  public double yVelocity = 0;

  private double originalX = 0;
  private double originalY = 0;

  Joystick m_Joystick = new Joystick(OIConstants.kDriverControllerPort);

  // The imu sensor
  public final StrafeOnTarget m_lock = new StrafeOnTarget();
  public final Pigeon2 m_imu = new Pigeon2(Constants.SwerveConstants.kIMU_ID);
  private final PhotonVisionGS m_vision;
  private final PhotonVisionGS2 m_vision2;
  public final AlignToClimb m_alignToClimb = new AlignToClimb();

  public RotationEnum isAutoRotate = RotationEnum.NONE;
  public boolean isAutoRotateToggle = true;
  
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    };
  }

  private final SwerveDrivePoseEstimator m_poseEstimator;
  
  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;

  // Create Field2d for robot and trajectory visualizations.
  public Field2d m_field;
  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(PhotonVisionGS m_vision, PhotonVisionGS2 m_vision2) {
    m_turnCtrl.setTolerance(10.00);
    this.m_vision = m_vision;
    this.m_vision2 = m_vision2;
    
    offset_FL = SwerveConstants.kFrontLeftMagEncoderOffsetDegrees_NoNo;
    offset_RL = SwerveConstants.kRearLeftMagEncoderOffsetDegrees_NoNo;
    offset_FR = SwerveConstants.kFrontRightMagEncoderOffsetDegrees_NoNo;
    offset_RR = SwerveConstants.kRearRightMagEncoderOffsetDegrees_NoNo;

  m_frontLeft =
    new SwerveModuleOffboard(
      SwerveConstants.kFrontLeftDriveMotorPort,
      SwerveConstants.kFrontLeftTurningMotorPort,
      SwerveConstants.kFrontLeftMagEncoderPort,
      offset_FL);

  m_rearLeft =
    new SwerveModuleOffboard(
      SwerveConstants.kRearLeftDriveMotorPort,
      SwerveConstants.kRearLeftTurningMotorPort,
      SwerveConstants.kRearLeftMagEncoderPort,
      offset_RL);

  m_frontRight =
    new SwerveModuleOffboard(
      SwerveConstants.kFrontRightDriveMotorPort,
      SwerveConstants.kFrontRightTurningMotorPort,
      SwerveConstants.kFrontRightMagEncoderPort,
      offset_FR);

  m_rearRight =
    new SwerveModuleOffboard(
      SwerveConstants.kRearRightDriveMotorPort,
      SwerveConstants.kRearRightTurningMotorPort,
      SwerveConstants.kRearRightMagEncoderPort,
      offset_RR);

  // Odometry class for tracking robot pose
  m_odometry =
      new SwerveDriveOdometry(
          SwerveConstants.kDriveKinematics,
          m_imu.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

          // Create and push Field2d to SmartDashboard.
    m_field = new Field2d();
    SmartDashboard.putData(m_field);

    // Configure the AutoBuilder last
      try {
      AutoBuilder.configure(
        this::getEstimatedPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
          new PIDConstants(7.0, 0.0, 0.0), // Translation PID constants
          new PIDConstants(Math.PI + Math.E - 2, 0.0, 0.0)), // Rotation PID constants
        RobotConfig.fromGUISettings(),
            ()->{
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
       // Reference to this subsystem to set requirements
    );
  } catch (Exception e) {
    DriverStation.reportError(e.getMessage(), e.getStackTrace());
  }
    // Reference: https://www.chiefdelphi.com/t/has-anyone-gotten-pathplanner-integrated-with-the-maxswerve-template/443646

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));

    SmartDashboard.putData(m_field);

    m_poseEstimator =
    new SwerveDrivePoseEstimator(
      Constants.SwerveConstants.kDriveKinematics,
      m_imu.getRotation2d(),
      getModulePositions(),
      getPose(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
    originalX = m_poseEstimator.getEstimatedPosition().getX();
    originalY = m_poseEstimator.getEstimatedPosition().getY();
    m_timerX.start();
    m_timerY.start();
    rotationTimer.start();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block

    if (m_vision.getTarget().map((t) -> t.getPoseAmbiguity()).orElse(1.0) <= .2 && m_vision.getTarget().map((t) -> t.getPoseAmbiguity()).orElse(1.0) <= .2) {
      if (m_vision.getTarget().map((t) -> t.getPoseAmbiguity()).orElse(1.0) <= m_vision2.getTarget().map((t) -> t.getPoseAmbiguity()).orElse(1.0)) {
        m_vision.getRobotPose().ifPresent((robotPose) -> m_poseEstimator.addVisionMeasurement(robotPose, m_vision.getApriltagTime()));
      }
      else {
        m_vision2.getRobotPose().ifPresent((robotPose) -> m_poseEstimator.addVisionMeasurement(robotPose, m_vision.getApriltagTime()));
      }
      }
    else if (m_vision.getTarget().map((t) -> t.getPoseAmbiguity()).orElse(1.0) <= .2) {
      m_vision.getRobotPose().ifPresent((robotPose) -> m_poseEstimator.addVisionMeasurement(robotPose, m_vision.getApriltagTime()));
    }
    else if (m_vision2.getTarget().map((t) -> t.getPoseAmbiguity()).orElse(1.0) < .2) {
        m_vision2.getRobotPose().ifPresent((robotPose) -> m_poseEstimator.addVisionMeasurement(robotPose, m_vision.getApriltagTime()));
    }

    m_poseEstimator.update(m_imu.getRotation2d(), getModulePositions());

    // Look into
    if (AutoCommandManager.isSim = false) {m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());}

    m_odometry.update(
        m_imu.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
    
    // Update robot position on Field2d.
    m_field.setRobotPose(getEstimatedPose());

    // SmartDashboard.putNumber("Gyro angle", m_imu.getYaw().getValueAsDouble());
    // SmartDashboard.putNumber("Gyro pitch", m_imu.getPitch().getValueAsDouble());
    // SmartDashboard.putNumber("Gyro roll", m_imu.getRoll().getValueAsDouble());

    SmartDashboard.putNumber("Estimated Pose X", m_poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Estimated Pose Y", m_poseEstimator.getEstimatedPosition().getY());

    SmartDashboard.putNumber("Pose Rotation", getPose().getRotation().getDegrees());

    // SmartDashboard.putNumber("Yep", m_frontLeft.getVelocity());

    pose_publisher.set(getPose());
    swerve_publisher.set(new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState() } ); // :3

    

    // Diagnostics

  if (Constants.kDebugLevel >=3) {
      SmartDashboard.putNumber("FL Mag Enc", m_frontLeft.getCanCoder());
      SmartDashboard.putNumber("FR Mag Enc", m_frontRight.getCanCoder());
      SmartDashboard.putNumber("RL Mag Enc", m_rearLeft.getCanCoder());
      SmartDashboard.putNumber("RR Mag Enc", m_rearRight.getCanCoder());

      SmartDashboard.putNumber("FL Angle State", m_frontLeft.getState().angle.getDegrees());
      SmartDashboard.putNumber("FL Angle SparkMax", m_frontLeft.getAngle().getDegrees());
      SmartDashboard.putNumber("FL Angle CanCoder", m_frontLeft.getCanCoder());
      SmartDashboard.putNumber("FL Angle Offset", m_frontLeft.getCanCoder() - m_frontLeft.getAngle().getDegrees());
      SmartDashboard.putNumber("FL Angle Current",m_frontLeft.getTurnCurrent());
      
      SmartDashboard.putNumber("FL Turn Enc", m_frontLeft.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("FR Turn Enc", m_frontRight.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("RL Turn Enc", m_rearLeft.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("RR Turn Enc", m_rearRight.getPosition().angle.getDegrees());

      SmartDashboard.putNumber("RL Drive encoder", m_rearLeft.getPosition().distanceMeters);
    }

    Vector<Double> robotVector = new Vector<>();
    if (Math.abs(xVelocity) <= 0.1) {
      robotVector.add(xVelocity);
    }
    else {
      robotVector.add(0.0);
    }
    if (Math.abs(yVelocity) <= 0.1) {
      robotVector.add(yVelocity);
    }
    else {
      robotVector.add(0.0);
    }

    if (isAutoRotate == RotationEnum.STRAFEONTARGET) {
      autoRotateSpeed = m_lock.execute(m_poseEstimator.getEstimatedPosition());
    }

    if (isAutoRotate == RotationEnum.STRAFEONTARGET && isAutoRotateToggle) {
      m_lock.initialize();
      isAutoRotateToggle = false;
    }

    else if (isAutoRotate == RotationEnum.NONE) {
      isAutoRotateToggle = true;
    }

    if (isAutoRotate == RotationEnum.ALIGNTOCLIMB) {
      autoRotateSpeed = m_alignToClimb.execute(getEstimatedPose());
    }

    if (isAutoRotate == RotationEnum.ALIGNTOCLIMB && isAutoRotateToggle) {
      m_alignToClimb.initialize();
      isAutoRotateToggle = false;
    }
    
    getRobotVelocityX();
    getRobotVelocityY();

    SmartDashboard.putNumber("Estimated rotation", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
}

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void zeroIMU() {
    m_imu.setYaw(0.0);
    m_poseEstimator.resetPosition(m_imu.getRotation2d(), getModulePositions(), getPose());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_imu.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
    m_poseEstimator.resetPosition(m_imu.getRotation2d(), getModulePositions(), pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    rot = isAutoRotate != RotationEnum.NONE ? autoRotateSpeed : rot;

    // Apply joystick deadband
    xSpeed = MathUtil.applyDeadband(xSpeed, OIConstants.kDeadband, 1.0);
    ySpeed = MathUtil.applyDeadband(ySpeed, OIConstants.kDeadband, 1.0);
    rot = isAutoRotate != RotationEnum.NONE ? rot : MathUtil.applyDeadband(rot, OIConstants.kRotationDeadband, 1.0);

    // xSpeed *= Constants.SwerveConstants.kMaxSpeedTeleop;
    // ySpeed *= Constants.SwerveConstants.kMaxSpeedTeleop;
    // rot *= Constants.ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND;

    // Apply speed scaling
    xSpeed = xSpeed * m_DriverSpeedScale;
    ySpeed = ySpeed * m_DriverSpeedScale;
    rot = rot * m_DriverSpeedScale;

    if (isAutoRotate == RotationEnum.STRAFEONTARGET) {
      fieldRelative = false;
    }

    /*if (isAutoRotate == false && Math.abs(rot / ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND) <= 0.1 
    && rotationTimer.hasElapsed(0.1)
    ) {
      if (roboNoSpino) {
        goalAngle = m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
        roboNoSpino = false;
      }
    
      m_turnCtrl.setSetpoint(goalAngle);
      double m_output = MathUtil.clamp(m_turnCtrl.calculate(m_poseEstimator.getEstimatedPosition().getRotation().getDegrees()), -1.0, 1.0);
      rot = m_output;
    }
    else {
      goalAngle = m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
      m_turnCtrl.reset();
      roboNoSpino = true;
      rotationTimer.restart();
    }
    SmartDashboard.putNumber("Goal Angle", goalAngle);
    */

    var swerveModuleStates =
        SwerveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_imu.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds( swerveModuleStates, SwerveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[SwerveConstants.kSwerveFL_enum]);
    m_frontRight.setDesiredState(swerveModuleStates[SwerveConstants.kSwerveFR_enum]);
    m_rearLeft.setDesiredState(swerveModuleStates[SwerveConstants.kSwerveRL_enum]);
    m_rearRight.setDesiredState(swerveModuleStates[SwerveConstants.kSwerveRR_enum]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
    desiredStates, SwerveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[SwerveConstants.kSwerveFL_enum]);
    m_frontRight.setDesiredState(desiredStates[SwerveConstants.kSwerveFR_enum]);
    m_rearLeft.setDesiredState(desiredStates[SwerveConstants.kSwerveRL_enum]);
    m_rearRight.setDesiredState(desiredStates[SwerveConstants.kSwerveRR_enum]);
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    this.drive(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,speeds.omegaRadiansPerSecond,false);
    SmartDashboard.putNumber("DriveVelX", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("DriveVelY", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("DriveRotZ", speeds.omegaRadiansPerSecond);
  }
  
  public ChassisSpeeds getRobotRelativeSpeeds(){
    return SwerveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(),
                                                           m_frontRight.getState(),
                                                           m_rearLeft.getState(),
                                                           m_rearRight.getState());
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoder();
    m_rearLeft.resetEncoder();
    m_frontRight.resetEncoder();
    m_rearRight.resetEncoder();
  }

  /* Sets how fast the human driver can drive */
  public void setMaxOutput(double val) {
    m_DriverSpeedScale = val;
  }

  public void toggleMaxOutput() {
    if (m_DriverSpeedScale == 1.0){
      m_DriverSpeedScale = Constants.kDriverSpeedLimit;
    } else {
      m_DriverSpeedScale = 1.0;
    }
  }

  public Pose2d getEstimatedPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void getRobotVelocityX() {
    if (m_timerX.hasElapsed(0.1)) {
      double newX = m_poseEstimator.getEstimatedPosition().getX();
      xVelocity = (newX - originalX) / m_timerX.get();
      originalX = newX;
      m_timerX.restart();
    }
  }

  public void getRobotVelocityY() {
    if (m_timerY.hasElapsed(0.1)) {
      double newY = m_poseEstimator.getEstimatedPosition().getY();
      yVelocity = (newY - originalY) / m_timerY.get();
      originalY = newY;
      m_timerY.restart();
    }
  }

  public void zeroGyro() {
    m_imu.setYaw(0);
    resetOdometry(new Pose2d());
  }

  public void setEstimatedPose(Pose2d pose) {
    m_poseEstimator.resetPosition(m_imu.getRotation2d(), getModulePositions(), pose);
  }

  public void zeroEstimatedPose() {
    m_poseEstimator.resetPosition(m_imu.getRotation2d(), getModulePositions(), new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
  }
}