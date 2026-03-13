// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib2202.builder.RobotContainer;
import frc.robot2026.RegisteredCommands;
import frc.robot2026.subsystems.Shooter.Targeter.driveMode;

// A command that should shoot and drive targetcentric while active
public class shootAndScoot extends Command {
  /** Creates a new shootAndScoot. */

  private Command shootCommand;

  public shootAndScoot() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shootCommand = RegisteredCommands.ncShoot();
    CommandScheduler.getInstance().schedule(shootCommand);
    CommandScheduler.getInstance().schedule(new setDriveMode(RobotContainer.getSubsystemOrNull("targeter"), driveMode.TargetCentric));
    System.out.println("***Shoot and scoot activated...");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().cancel(shootCommand);
    CommandScheduler.getInstance().schedule(new setDriveMode(RobotContainer.getSubsystemOrNull("targeter"), driveMode.FieldCentric));
    System.out.println("***Shoot and scoot done...");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
