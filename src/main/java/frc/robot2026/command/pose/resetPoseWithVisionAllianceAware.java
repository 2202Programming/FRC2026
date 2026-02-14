// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.command.pose;

import static edu.wpi.first.units.Units.Rotation;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2026.subsystems.Photonvision;
import frc.robot2026.subsystems.VisionPoseEstimator;

//This command should wait to see X number of targets on PV, and then get the average robot orientation from PV, and reset the gryo to this
//And add 180 deg if red alliance
//Should be the automatic version of the old AllianceAwareGryoReset

public class resetPoseWithVisionAllianceAware extends Command {
  /** Creates a new resetPoseWithVIsionAllianceAware. */

  private boolean resetDone;
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
    resetDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (pv.totalTargetsAllCameras() > 0) {
      System.out.println("***Vision pose gryo Pre reset is: " + vpe.getPose().getRotation().getDegrees());
      Rotation2d tempRot = pv.getAverageRot();
      if (DriverStation.getAlliance().get() == Alliance.Red)
        tempRot.rotateBy(new Rotation2d(Math.PI));
      vpe.setAnglePose(tempRot);
      resetDone = true;
      System.out.println("***Vision pose gryo reset done, set to: " + vpe.getPose().getRotation().getDegrees());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return resetDone;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true; // Allows the command to run when disabled
  }
}
