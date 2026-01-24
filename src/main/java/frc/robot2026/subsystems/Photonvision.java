// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.builder.Robot;
import frc.lib2202.command.WatcherCmd;
import static frc.robot2026.Constants.Vision.*;

public class Photonvision extends SubsystemBase {
  /** Creates a new Photonvision. */

  PhotonCamera camera;
  List<PhotonPipelineResult> results;
  PhotonPipelineResult lastResult;
  boolean hasTargets;
  boolean multiTag;
  List<PhotonTrackedTarget> targets;
  private final PhotonPoseEstimator photonEstimator;
  Pose2d currentPose;

  public Photonvision() {
    setName("photonvision");

    camera = new PhotonCamera("HD_USB_Camera");
    // auto start the watcher
    getWatcherCmd();
    photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
  }

  @Override
  public void periodic() {
    targets = null;
    // This method will be called once per scheduler run
    // Query the latest result from PhotonVision
    results = camera.getAllUnreadResults(); // docs say this is preferred, other call deprecated
    int lastIdx = results.size();
    if (lastIdx > 0) {
      lastResult = results.get(lastIdx - 1);
      hasTargets = lastResult.hasTargets();
      // Get a list of currently tracked targets.
      targets = lastResult.getTargets();

      process(targets);
    }

    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var result : camera.getAllUnreadResults()) {
      multiTag = true;
      visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
      if (visionEst.isEmpty()) {
        multiTag = false;
        visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
      }
      visionEst.ifPresent(
          est -> {
            currentPose = est.estimatedPose.toPose2d();
          });
    }

  }

  void process(List<PhotonTrackedTarget> targets) {
    // todo
  }

  public int howManyTargets() {
    if (targets == null)
      return -1; // targets seems like it can be null, protect - dpl
    return targets.size();
  }

  public double getCurrentPoseX() {
    if (currentPose == null)
      return -1;
    return currentPose.getX();
  }

  public double getCurrentPoseY() {
    if (currentPose == null)
      return -1;
    return currentPose.getY();
  }

  public boolean hasMultitarget(){
    return multiTag;
  }

  // Add a watcher so we can see stuff on network tables
  public WatcherCmd getWatcherCmd() {
    return this.new PhotonWatcher();
  }

  class PhotonWatcher extends WatcherCmd {
    PhotonWatcher() {
      addEntry("Photon_Has_Target", Photonvision.this::howManyTargets);
      addEntry("Photon Estimate X", Photonvision.this::getCurrentPoseX);
      addEntry("Photon Estimate Y", Photonvision.this::getCurrentPoseY);
      addEntry("Photon_Multi_Target", Photonvision.this::hasMultitarget);
    }
  }
}
