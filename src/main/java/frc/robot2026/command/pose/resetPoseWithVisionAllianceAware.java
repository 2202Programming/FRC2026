// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.command.pose;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2026.subsystems.Photonvision;
import frc.robot2026.subsystems.VisionPoseEstimator;

//This command should do a one-time gyro reset based on apriltags to align gyro with field heading.  
//Will do a low-accuracy single-tag estimate of rotation if that's all it can see, but will keep waiting for a multitag estimate before finally finishing.
//Should not need to be alliance-aware.

public class resetPoseWithVisionAllianceAware extends Command {
  /** Creates a new resetPoseWithVIsionAllianceAware. */

  private boolean singleResetDone;
  private boolean multiResetDone;
  private VisionPoseEstimator vpe;
  private Photonvision pv;

  public resetPoseWithVisionAllianceAware() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pv = RobotContainer.getSubsystem("photonvision");
    vpe = RobotContainer.getSubsystem("vision_odo");
    multiResetDone = false;
    singleResetDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (pv.anyMultiTags()) { // someone has a multitag
      for (int i = 0; i < pv.howManyCameras(); i++) {
        if (pv.hasMultitarget(i)) { // camera has a multitag, should be most reliable
          System.out.println("***Vision pose gryo Pre multitag reset is: " + vpe.getPose().getRotation().getDegrees());
          Rotation2d tempRot = pv.getCameraPose(i).getRotation();
          vpe.setAnglePose(tempRot);
          multiResetDone = true;
          System.out
              .println("***Vision pose gryo multitag reset done, set to: " + vpe.getPose().getRotation().getDegrees());
        }
      }
    }

    // there is no multitag yet, but one or more cameras have a single tag,
    // and we haven't done a single tag estimate yet
    if (pv.totalTargetsAllCameras() > 0 && !multiResetDone && !singleResetDone) {
      System.out.println("***Vision pose gryo Pre single tag reset is: " + vpe.getPose().getRotation().getDegrees());
      Rotation2d tempRot = pv.getAverageRot(); //there may be more than one camera with single tag, take an average of their rotation estimates.
      vpe.setAnglePose(tempRot);
      singleResetDone = true;
      System.out
          .println("***Vision pose gryo single tag reset done, set to: " + vpe.getPose().getRotation().getDegrees());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return multiResetDone;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true; // Allows the command to run when disabled
  }
}
