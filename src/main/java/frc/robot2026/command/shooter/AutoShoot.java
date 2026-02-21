// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.command.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot2026.subsystems.Shooter.Indexer;
import frc.robot2026.subsystems.Shooter.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoShoot extends Command {

  final Shooter shooter;
  final Indexer indexer;

  DoubleSupplier speedProvider;
  double idxPct;

  public AutoShoot(Shooter shooter, Indexer indexer, DoubleSupplier speed, double idxPct) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.speedProvider = speedProvider;
    this.idxPct = idxPct;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.flywheel.setSetpoint(speedProvider.getAsDouble());

    if(shooter.atSetpoint()) {
      indexer.setPct(idxPct);
    } else {
      indexer.setPct(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.flywheel.setSetpoint(0.0);
    indexer.setPct(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
