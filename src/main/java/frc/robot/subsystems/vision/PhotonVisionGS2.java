// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionGS2 extends SubsystemBase {
  private PhotonCamera camera = new PhotonCamera("Camera_Module_v2");
  private Rotation3d rd = new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(0));
  private Transform3d td = new Transform3d(0.305, 0.305, 0.305, rd);
  private Pose3d targetTd;
  private double apriltagTime; 
  public double distanceToApriltag = 0;

  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private PhotonPipelineResult result;
  private PhotonTrackedTarget target;

  private double ID = 0;
  private Debouncer m_filterSpeakerInView = new Debouncer (0.1, Debouncer.DebounceType.kBoth);
  private boolean speakerInView;
  private boolean speakerInView_filtered;

  private LinearFilter m_lowpass = LinearFilter.movingAverage(100);
  private double tx_out;
  //**heightMatters is the height of the object based on the april tags and the camera used for cacluations in shooting**//
  private double heightMatters = 1.93;
  // private double heightMatters = 2.02;
  public double m_goalAngle;

  public PhotonVisionGS2() {
    ID = 0;
  }


  @Override
  public void periodic() {
    result = camera.getLatestResult();
    apriltagTime = result.getTimestampSeconds();
    result.getTargets();

    if (result.hasTargets()) {
      PhotonTrackedTarget localTarget = result.getBestTarget();

      // Start of check list
      boolean foundSpeaker = true;
      // for (var thisTarget : result.targets) {  // Java 'for each' loop
      //   int myID = thisTarget.getFiducialId();
        // if ((myID == 4) || (myID == 7)) {
        //   foundSpeaker = true;
        // }
        // //they have -31 for height of camera
        // if (myID >= 11 && myID <= 16){
        //   //this is chian thingy
        //   //heightMatters = 1.35;
        // }
        // else if (myID == 5 || myID == 6){
        //   //this is amp
        //   //heightMatters = .61;
        // }
        // else if (myID == 8 || myID == 7 || myID == 3 || myID == 4){
        //   //heightMatters = 2.025;
        //   //this is speaker
        // }
        //else {heightMatters = -1;}
      // }

      Transform3d cameraToTarget = localTarget.getBestCameraToTarget();

      Pose3d aprilTagPose3d = aprilTagFieldLayout.getTagPose(localTarget.getFiducialId()).get();

      targetTd = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, aprilTagPose3d, td);

      ID = localTarget.getFiducialId();

      // if(ID == 3){ID = 4;}

      // if ((ID == 4) || (ID == 7)) { //blue-7 red-4
      target = localTarget;
      SmartDashboard.putNumber("April tag local targer number", localTarget.getFiducialId());

      if (foundSpeaker) {
        speakerInView = true;
        double yaw = (PhotonUtils.getYawToPose(targetTd.toPose2d(), aprilTagPose3d.toPose2d()).getDegrees());
        yaw *= ID == 7 ? 1 : -1;
        distanceToApriltag = PhotonUtils.getDistanceToPose(targetTd.toPose2d(), aprilTagPose3d.toPose2d());
        // SmartDashboard.putNumber("yaw", yaw);

        tx_out = yaw; //m_lowpass.calculate(yaw);

      } else {
        speakerInView = false;
        // m_lowpass.reset();
      }
      speakerInView_filtered = m_filterSpeakerInView.calculate(speakerInView);

  // SmartDashboard.putNumber("Height",heightMatters);

  double yaw = Units.radiansToDegrees(targetTd.getRotation().getZ());
  tx_out = m_lowpass.calculate(yaw);
  SmartDashboard.putNumber("April tag target number", target.getFiducialId());
  SmartDashboard.putNumber("April tag targetTd number", targetTd.getX());
} else {
  ID = 0;
  targetTd = null;
  target = null;
}
  // SmartDashboard.putNumber("April tag target number", target.getFiducialId());
  // SmartDashboard.putNumber("April tag targetTd number", targetTd.getX());
}
  // port: http://photonvision.local:5800

  public boolean isSpeakerInView() {
    return speakerInView_filtered;
  }

  public double getTargetHorzAngle() {
    return tx_out;
  }

  public double getTargetHeight() {
    return heightMatters;
  }

  public double getTargetDistance() {
    return distanceToApriltag;
  }

  public Optional <Double> getTargetYDistance() {
    return Optional.ofNullable(targetTd).map((t) -> t.getY());
  }

  public Optional <PhotonTrackedTarget> getTarget() {
    return Optional.ofNullable(target);
  }

  public Optional <Pose2d> getRobotPose() {
    return Optional.ofNullable(targetTd).map((t) -> t.toPose2d());
  }

  public double getApriltagTime() {
    return apriltagTime;
  }
}