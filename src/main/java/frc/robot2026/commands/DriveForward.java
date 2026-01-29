// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.command.pathing;

import java.io.IOException;
import java.nio.file.Path;
import java.text.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.util.Units;


import edu.wpi.first.wpilibj2.command.Command;

import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;

/*
 * 
 * @deprecated - use MoveToPose(), does same thing but has better error handling
 * and constraint handling. Also handles interrupt of command.
 * Folllows class naming conventions too.
 * 
 *  {@link #MoveToPose()} instead.
 */
@Deprecated
public class DriveForward extends Command {

  private Pose2d targetPose;
  private DriveTrainInterface m_Drivetrain;
  private PathConstraints pathConstraints;
  PathPlannerPath myPath;

  /** Creates a new driveToPose. */
  public DriveForward(Pose2d targetPose) {

    // this.targetPose = targetPose;
    m_Drivetrain = RobotContainer.getSubsystem("drivetrain");

    addRequirements(m_Drivetrain);
    try {
      myPath = PathPlannerPath.fromPathFile("Move_Forward");
    } catch (Exception f) {
      System.out.println("Couldn't find the path");
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pathConstraints = new PathConstraints(1.0, 1.0, Units.degreesToRadians(360), Units.degreesToRadians(540));

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // fix cmd.schedule() being deprecated
    AutoBuilder.followPath(myPath).schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
