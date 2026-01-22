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
  PhotonPipelineResult result;
  boolean hasTargets;
  List<PhotonTrackedTarget> targets;

  public Photonvision() {
    camera = new PhotonCamera("HD_USB_Camera");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
                // Query the latest result from PhotonVision
      result = camera.getLatestResult();
      hasTargets = result.hasTargets();
      // Get a list of currently tracked targets.
      targets = result.getTargets();
  }

  public int howManyTargets(){
    return targets.size();
  }

  // Add a watcher so we can see stuff on network tables
  public WatcherCmd getWatcherCmd() {
    return this.new PhotonWatcher();
  }


  class PhotonWatcher extends WatcherCmd {  
    PhotonWatcher() {
       addEntry("Photon_Has_Target", Photonvision.this::howManyTargets, 1);
    }
  }
}
