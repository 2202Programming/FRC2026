// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.robot2026.command.pathing.goDistance;
import frc.robot2026.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoClimberCommand extends SequentialCommandGroup {
  /** Creates a new autoClimberCommand. */
  Climber climber;
  SwerveDrivetrain dt;


  public autoClimberCommand(boolean leftSide) {
    climber = RobotContainer.getSubsystem(Climber.class);
    dt = RobotContainer.getSubsystem(SwerveDrivetrain.class);
    

    addRequirements(climber, dt);
    // Use addRequirements() here to declare subsystem dependencies
    if (leftSide) {
      addCommands(climber.armsToPoint(climber.climbposition()), new goDistance(0), climber.armsToPoint(0));
    } else {
      addCommands(climber.armsToPoint(climber.climbposition()), new goDistance(0), climber.armsToPoint(0));
    }
    
    // order of opp: send arms to a position (predetermined by the subsystem), dive backwards X amount, then arms to position 0.)
    //The command itself is not difficult, the issues come other forms.


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
  }
}
