// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.command;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.Sensors;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.robot2026.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoClimberCommand extends SequentialCommandGroup {
  /** Creates a new autoClimberCommand. */
  Climber climber;
  SwerveDrivetrain dt;
  Sensors sensor;


  public autoClimberCommand(boolean leftSide) {
    climber = RobotContainer.getSubsystem(Climber.class);
    dt = RobotContainer.getSubsystem(SwerveDrivetrain.class);
    sensor = RobotContainer.getSubsystem("sensors");
    

    addRequirements(climber, dt,sensor);
    // Use addRequirements() here to declare subsystem dependencies

    addCommands(climber.armsToPoint(climber.climbposition()), new climberManuver(leftSide), climber.armsToPoint(0));
    
    // order of opp: send arms to a position (predetermined by the subsystem), dive backwards X amount, then arms to position 0.)
    //The command itself is not difficult, the issues come other forms.


    
  }
}
