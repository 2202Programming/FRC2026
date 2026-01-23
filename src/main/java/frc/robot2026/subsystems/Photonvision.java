// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.command.WatcherCmd;

public class Photonvision extends SubsystemBase {
  /** Creates a new Photonvision. */

  PhotonCamera camera;
  List<PhotonPipelineResult> results;
  PhotonPipelineResult lastResult;
  boolean hasTargets;
  List<PhotonTrackedTarget> targets;

  public Photonvision() {
    setName("photonvision");
    camera = new PhotonCamera("HD_USB_Camera");
    // auto start the watcher
    getWatcherCmd();
  }

  @Override
  public void periodic() {
      targets = null;
      // This method will be called once per scheduler run
      // Query the latest result from PhotonVision
      results = camera.getAllUnreadResults(); //docs say this is preferred, other call deprecated
      int lastIdx = results.size();
      if (lastIdx > 0) { 
        lastResult = results.get( lastIdx - 1);
        hasTargets = lastResult.hasTargets();
        // Get a list of currently tracked targets.
        targets = lastResult.getTargets();

        process(targets);
      }
  }

  void process(List<PhotonTrackedTarget> targets) {
    //todo 
  }

  public int howManyTargets(){
    if (targets == null) return -1;  // targets seems like it can be null, protect - dpl
    return targets.size();
  }

  // Add a watcher so we can see stuff on network tables
  public WatcherCmd getWatcherCmd() {
    return this.new PhotonWatcher();
  }


  class PhotonWatcher extends WatcherCmd {  
    PhotonWatcher() {
       addEntry("Photon_Has_Target", Photonvision.this::howManyTargets);
    }
  }
}
