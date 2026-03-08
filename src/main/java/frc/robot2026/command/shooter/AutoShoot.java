// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.command.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib2202.builder.RobotContainer;
import frc.robot2026.subsystems.Shooter.Indexer;
import frc.robot2026.subsystems.Shooter.Shooter;

public class AutoShoot extends Command {
  static boolean left_active = true;    //default to left first

  final Shooter shooter;
  final Indexer indexer;

  final DoubleSupplier speedProvider;
  final DoubleSupplier toleranceProvider;
  final double idxPct;
  final double idxLoad = 0.3; // loading speed, load to gate
  final boolean is_left;

  // state vars
  boolean gate, gate_prev; // gate edge
  int shots_taken;

  public AutoShoot(String side, DoubleSupplier speedProvider, DoubleSupplier toleranceProvider, double idxPct) {
    // Shooter shooter, Indexer indexer, double idxPct) {
    this.shooter = RobotContainer.getSubsystem("shooter_" + side);
    this.indexer = RobotContainer.getSubsystem("indexer_" + side);
    // Targeter targeter = RobotContainer.getSubsystem(Targeter.class);
    this.speedProvider = speedProvider;
    this.toleranceProvider = toleranceProvider;
    this.idxPct = idxPct;

    this.is_left = side.startsWith("l");
    addRequirements(shooter, indexer);
    setName("AutoShoot_" + side);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shots_taken = 0;
    // gates may be false because fuel was backed off behind gate to keep flywheel
    // clear
    gate = gate_prev = indexer.hasFuel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double idxCmd = 0.0;
    shooter.flywheel.setSetpoint(speedProvider.getAsDouble() * shooter.getSpeedFactor() );
    shooter.flywheel.setVelocityTolerance(toleranceProvider.getAsDouble());
    gate = indexer.hasFuel();

    boolean myturn = is_left  ?  AutoShoot.left_active : !AutoShoot.left_active;

    if (shooter.atSetpoint() && myturn) {
      indexer.setPct(idxPct);
    } else {
      // roll indexer until we have fuel
      idxCmd = (gate) ? 0.0 : idxLoad;
      indexer.setPct(idxCmd);
    }

    // look for High->Low on gate to count shots
    if (gate_prev && (gate != gate_prev)) {
      // should be fuel leaving the bot, count it
      shots_taken++;
      AutoShoot.left_active = ! AutoShoot.left_active;
    } else  //if we don't have fuel, switch turns
    if (myturn && !gate){
       AutoShoot.left_active = ! AutoShoot.left_active;
    }

    // low to high indicates loading/readying a shot
    // if (gate && (gate != gate_prev)) { }

    gate_prev = gate;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.setPct(0.0);
    shooter.addShots(shots_taken);
    // leave shooter running for 300ms after indexer is off
    // to make sure it is cleared
    double spd = speedProvider.getAsDouble();
    // create cmd and schedule it before we leave, 0.0 set at end
    var cmd = shooter.cmdVelocityDuration(spd, 0.300);
    CommandScheduler.getInstance().schedule(cmd);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
