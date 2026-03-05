// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.command;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib2202.builder.RobotContainer;
import frc.robot2026.subsystems.Hopper;
import frc.robot2026.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Agitator extends SequentialCommandGroup {

  public Agitator() {

    Intake intake = RobotContainer.getSubsystem("intake");
    Hopper hopper = RobotContainer.getSubsystem(Hopper.class);
    addCommands(
        hopper.cmdBeltPct(0.65),
        intake.cmdPctPwr(0.65),
        new WaitCommand(0.5),
        hopper.cmdBeltPct(-0.65),
        intake.cmdPctPwr(-0.65),
        new WaitCommand(0.5),
        hopper.cmdBeltPct(0.65),
        intake.cmdPctPwr(-0.65),
        new WaitCommand(0.5),
        hopper.cmdBeltPct(-0.65),
        intake.cmdPctPwr(0.65),
        new WaitCommand(0.5));

    addRequirements(intake, hopper);
  }
  
}
