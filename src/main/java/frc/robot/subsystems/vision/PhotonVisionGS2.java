// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.isInAreaEnum;
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

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionGS2 extends SubsystemBase {
  // private PhotonCamera camera = new PhotonCamera("Camera 2");
  // private Rotation3d rd = new Rotation3d(Units.degreesToRadians(2.6), Units.degreesToRadians(0), Units.degreesToRadians(190.4));
  // private Transform3d td = new Transform3d(-0.29, -0.285, 0.400, rd);
  // private Pose3d targetTd;
  // private double apriltagTime; 
  // public double distanceToApriltag = 0;

  // private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();
  // private PhotonPipelineResult result;
  // private PhotonTrackedTarget target;

  // private double ID = 0;
  // private Debouncer m_filterSpeakerInView = new Debouncer (0.1, Debouncer.DebounceType.kBoth);
  // private boolean speakerInView;
  // private boolean speakerInView_filtered;

  // private LinearFilter m_lowpass = LinearFilter.movingAverage(100);
  // private double tx_out;
  // //**heightMatters is the height of the object based on the april tags and the camera used for cacluations in shooting**//
  // private double heightMatters = 1.93;
  // // private double heightMatters = 2.02;
  // public double m_goalAngle;

  public PhotonVisionGS2() {
    // ID = 0;
  }


//   @Override
//   public void periodic() {
//     result = camera.getLatestResult();
//     apriltagTime = result.getTimestampSeconds();
//     result.getTargets();
//     camera.setPipelineIndex(0);

//     if (result.hasTargets()) {
//       PhotonTrackedTarget localTarget;
//       localTarget = result.getTargets().stream()
//         .filter(((t) -> t.getFiducialId() == isInAreaEnum.areaEnum.getAprilTag()))
//         .findAny()
//         .orElseGet((() -> result.getBestTarget()));
//       // Start of check list
//       boolean foundSpeaker = true;

//       Transform3d cameraToTarget = localTarget.getBestCameraToTarget();
//       var id = localTarget == null ? null : localTarget.getFiducialId();
//       Pose3d aprilTagPose3d = aprilTagFieldLayout.getTagPose(id).get();

//       targetTd = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, aprilTagPose3d, td);

//       ID = localTarget.getFiducialId();

//       target = localTarget;
//       // SmartDashboard.putNumber("April tag local targer number", localTarget.getFiducialId());

//       if (foundSpeaker) {
//         speakerInView = true;
//         double yaw = (PhotonUtils.getYawToPose(targetTd.toPose2d(), aprilTagPose3d.toPose2d()).getDegrees());
//         yaw *= ID == 7 ? 1 : -1;
//         distanceToApriltag = PhotonUtils.getDistanceToPose(targetTd.toPose2d(), aprilTagPose3d.toPose2d());
//         tx_out = yaw; //m_lowpass.calculate(yaw);
//       } else {
//         speakerInView = false;
//         // m_lowpass.reset();
//       }
//       speakerInView_filtered = m_filterSpeakerInView.calculate(speakerInView);

//   double yaw = Units.radiansToDegrees(targetTd.getRotation().getZ());
//   tx_out = m_lowpass.calculate(yaw);
//   // SmartDashboard.putNumber("April tag target number", target.getFiducialId());
//   // SmartDashboard.putNumber("April tag targetTd number", targetTd.getX());
// } else {
//   ID = 0;
//   targetTd = null;
//   target = null;
//   }
// }
//   // port: http://photonvision.local:5800

//   public boolean isSpeakerInView() {
//     return speakerInView_filtered;
//   }

//   public double getTargetHorzAngle() {
//     return tx_out;
//   }

//   public double getTargetHeight() {
//     return heightMatters;
//   }

//   public double getTargetDistance() {
//     return distanceToApriltag;
//   }

//   public Optional <Double> getTargetYDistance() {
//     return Optional.ofNullable(targetTd).map((t) -> t.getY());
//   }

//   public Optional <PhotonTrackedTarget> getTarget() {
//     return Optional.ofNullable(target);
//   }

//   public Optional <Pose2d> getRobotPose() {
//     return Optional.ofNullable(targetTd).map((t) -> t.toPose2d());
//   }

//   public double getApriltagTime() {
//     return apriltagTime;
//   }
}