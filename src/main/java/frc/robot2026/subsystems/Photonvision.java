// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot2026.Constants.Vision;

import static frc.robot2026.Constants.Vision.*;

import frc.lib2202.command.WatcherCmd;

//individual photonvision USB cameras
class RobotCamera {

  

  PhotonCamera camera;
  List<PhotonPipelineResult> results;
  PhotonPipelineResult lastResult;
  boolean hasTargets;
  boolean multiTag;
  List<PhotonTrackedTarget> targets;
  private final PhotonPoseEstimator photonEstimator;
  Pose2d currentPose;
  int camera_number;

  public RobotCamera(String name, int camera_number) {
    camera = new PhotonCamera(name);
    this.camera_number = camera_number;
    photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam[camera_number]);
  }

  public void update() {
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

    }

    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var result : camera.getAllUnreadResults()) {
      multiTag = true;
      visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
      if (visionEst.isEmpty()) { //less than 2 tages, no multitag available
        multiTag = false;
        visionEst = photonEstimator.estimateLowestAmbiguityPose(result); //use single tag estimator
      }
      visionEst.ifPresent(
          est -> {
            currentPose = est.estimatedPose.toPose2d();
          });
    }

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

  public boolean hasMultitarget() {
    return multiTag;
  }

}

public class Photonvision extends SubsystemBase {
  /** Creates a new Photonvision. */

  List<RobotCamera> camerasList = new ArrayList<RobotCamera>();
  List<Integer> Photon_How_Many_Targets = new ArrayList<Integer>();
  List<Boolean> Photon_Has_Multi_Target = new ArrayList<Boolean>();
  List<Double> PoseX = new ArrayList<Double>();
  List<Double> PoseY = new ArrayList<Double>();

  public Photonvision() {
    setName("photonvision");

    for (int i = 0; i < Vision.CAMERA_NAMES.length; i++) {
      camerasList.add(new RobotCamera(Vision.CAMERA_NAMES[i], i));
      Photon_Has_Multi_Target.add(false);
      Photon_How_Many_Targets.add(-1);
      PoseX.add(-1.0);
      PoseY.add(-1.0);
    }
    getWatcherCmd();

  }

  @Override
  public void periodic() {
    RobotCamera currentCamera;
    for (int i = 0; i < camerasList.size(); i++) {
      currentCamera = camerasList.get(i);
      currentCamera.update(); //run each camera's periodic
      Photon_How_Many_Targets.set(i, currentCamera.howManyTargets());
      Photon_Has_Multi_Target.set(i, currentCamera.hasMultitarget());
      PoseX.set(i, currentCamera.getCurrentPoseX());
      PoseY.set(i, currentCamera.getCurrentPoseY());
    }
  }

  void process(List<PhotonTrackedTarget> targets) {
    // todo
  }

  // Add a watcher so we can see stuff on network tables
  public WatcherCmd getWatcherCmd() {
    return this.new PhotonWatcher();
  }

  public int howManyTargets(int listPos) {
    return camerasList.get(listPos).howManyTargets();
  }

  public boolean hasMultitarget(int listPos) {
    return camerasList.get(listPos).hasMultitarget();
  }

  public double getPoseX(int listPos) {
    return camerasList.get(listPos).getCurrentPoseX();
  }

  public double getPoseY(int listPos) {
    return camerasList.get(listPos).getCurrentPoseY();
  }

  class PhotonWatcher extends WatcherCmd {
    PhotonWatcher() {
      RobotCamera currentCamera;
      for (int i = 0; i < Vision.CAMERA_NAMES.length; i++) {
        currentCamera = camerasList.get(i);
        addEntry("Photon_How_Many_Targets[Cam" + i + "]", currentCamera::howManyTargets);
        addEntry("Photon Estimate X[Cam" + i + "]", currentCamera::getCurrentPoseX);
        addEntry("Photon Estimate Y[Cam" + i + "]", currentCamera::getCurrentPoseY);
        addEntry("Photon_Multi_Target[Cam" + i + "]", currentCamera::hasMultitarget);
      }
    }
  }
}
