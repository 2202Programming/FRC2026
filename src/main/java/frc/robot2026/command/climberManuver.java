// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.command;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.subsystem.swerve.IHeadingProvider;

// WARNING WARNING WARNING
// Ok, this code is only intended to be used with the autoClimberCommand sequence. It has no requirements and should not be touched by anything else. 
// Consider yourself warned


public class climberManuver extends Command {

//left and right defined from driver persepective to the tower


  public final double lxStart = 0.5; //CANNOT STRESS HOW MUCH OF A GUESS THESE ARE.
  public final double rxStart = 1.5; //TODO get actual values and put in constants
  public final double ly = 5.0;
  public final double ry = 3.0;
  public final double xEnd = 1.0;

  public final Pose2d lStartPose = new Pose2d(lxStart,ly, Rotation2d.fromDegrees(0.0)); 
  public final Pose2d rStartPose = new Pose2d(rxStart,ry, Rotation2d.fromDegrees(180.0));
  private Rotation2d endRot;

  boolean leftSide;

  PathPlannerPath path;
  Command runPath;

  public climberManuver(boolean leftSide) {
        // Create a list of waypoints from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not
    // use holonomic rotation.
    this.leftSide = leftSide;
    try {
      IHeadingProvider sensor = RobotContainer.getSubsystem("sensors");
      endRot = sensor.getHeading();
    } catch (Exception e) {
      System.out.println("Current bot does not have a sensor. THE HELL ARE YOU DOING HOW DID YOU GET HERE");
      endRot = Rotation2d.fromDegrees((leftSide?0.0:180.0)); //THIS IS SO BAD
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        (leftSide ? lStartPose : rStartPose),
        new Pose2d(xEnd, (leftSide?ly:ry), endRot)); // TODO THIS IS BAD. Should not reference like this.

    RobotLimits limits = RobotContainer.getRobotSpecs().getRobotLimits();
    PathConstraints constraints =  new PathConstraints(limits.kMaxSpeed, limits.kMaxSpeed / 1.33, 
                              limits.kMaxAngularSpeed, limits.kMaxAngularSpeed / 0.75); //pulled from the MoveToPose Command
    // You can also use unlimited constraints, only limited by motor torque and
    // nominal battery voltage

    // Create the path using the waypoints created above
    path = new PathPlannerPath(
        waypoints,
        constraints,
        null, // The ideal starting state, this is only relevant for pre-planned paths, so can
              // be null for on-the-fly paths.
        new GoalEndState(0.0, endRot) // Goal end state. You can set a holonomic rotation here. If

    );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;
    runPath = AutoBuilder.followPath(path);
  }

  public boolean isFinished() {
    return runPath.isFinished();
  }
}
