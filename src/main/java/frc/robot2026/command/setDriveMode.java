// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.command;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib2202.command.swerve.FieldCentricDrive;
import frc.lib2202.command.swerve.TargetCentricDrive;
import frc.robot2026.subsystems.Shooter.Targeter;
import frc.robot2026.subsystems.Shooter.Targeter.driveMode;

// Sets drive mode
public class setDriveMode extends InstantCommand {
  private Targeter m_targeter;
  private driveMode requestedMode;

  public setDriveMode(Targeter m_targeter, driveMode requestedMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_targeter = m_targeter;
    this.requestedMode = requestedMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (requestedMode) {
      case FieldCentric: {
        CommandScheduler.getInstance().schedule(new FieldCentricDrive());
        m_targeter.currentDriveMode = driveMode.FieldCentric;
        System.out.println("*** Switching to Field Centric Drive Mode ***");
      }
      case TargetCentric: {
        CommandScheduler.getInstance()
            .schedule(new TargetCentricDrive(m_targeter.redHubTarget, m_targeter.blueHubTarget));
        m_targeter.currentDriveMode = driveMode.TargetCentric;
        System.out.println("*** Switching to Target Centric Drive Mode ***");
      }
    }
  }

}
