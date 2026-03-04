// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.command;

import java.util.Optional;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.command.pathing.MoveToPose;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.robot2026.Constants.TheField;
import frc.robot2026.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoClimberCommand extends SequentialCommandGroup {
  /** Creates a new autoClimberCommand. */
  Climber climber;
  DriveTrainInterface sdt;

  final Pose3d realCenter;

  // Blue position for auto climbing.
  final Pose2d leftPose;
  final Pose2d rightPose;

  public autoClimberCommand(boolean leftSide) {

    Optional<Pose3d> BlueCenter = TheField.fieldLayout.getTagPose(31);
    Optional<Pose3d> RedCenter = TheField.fieldLayout.getTagPose(16);
    if (BlueCenter.isPresent()) {
      realCenter = BlueCenter.get();
    } else if (RedCenter.isPresent()) {
      realCenter = RedCenter.get();
    } else {
      realCenter = null;
      System.out.println("No center found");
    }

    leftPose = realCenter.toPose2d()
        .transformBy(new Transform2d(new Translation2d(0.5, 1.0), Rotation2d.fromDegrees(0.0)));
    rightPose = realCenter.toPose2d()
        .transformBy(new Transform2d(new Translation2d(1.5, -1.0), Rotation2d.fromDegrees(180.0)));

    climber = RobotContainer.getSubsystem("climber");
    sdt = RobotContainer.getSubsystem("drivetrain");

    RobotLimits limits = RobotContainer.getRobotSpecs().getRobotLimits();
    PathConstraints constraints = new PathConstraints(limits.kMaxSpeed, limits.kMaxSpeed / 1.33,
        limits.kMaxAngularSpeed, limits.kMaxAngularSpeed / 0.75);

    addRequirements(climber, sdt);
    // Use addRequirements() here to declare subsystem dependencies
    addCommands(new MoveToPose("vision_odo",
        constraints,
        (leftSide ? leftPose : rightPose)),
        climber.armsToPoint(Climber.ExtendPosition),
        new climberManuver(leftSide),
        climber.armsToPoint(0));

    // Things to add / fix: Make sure the distances are correct in manuver, switch
    // around left and right side, make climb position run in parallel with movement

    // order of opp: send arms to a position (predetermined by the subsystem), dive
    // backwards X amount, then arms to position 0.)
    // The command itself is not difficult, the issues come other forms.
  }
}
