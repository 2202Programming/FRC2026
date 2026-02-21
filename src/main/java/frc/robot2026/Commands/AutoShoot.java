// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2026.subsystems.Shooter.Indexer;
import frc.robot2026.subsystems.Shooter.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoShoot extends Command {

  final Indexer indexer;
  final Shooter shooter;

  double speed;
  boolean repeat;
  int framesTol;

  int count = 0;

  Phase phase;
  boolean ballReady;

  public enum Phase{
    ReadyToShoot, Shooting, Finished;
  }

  /**
   * @param speed The speed, in m/s of the flywheel.
   * @param repeat If the command should be repeated on an interval.
   * @param interval How often the command fires, in robot frames.
   */
  public AutoShoot(double speed, boolean repeat, int framesTol) {
    this.speed = speed;
    this.repeat = repeat;
    this.framesTol = framesTol;
    this.shooter = RobotContainer.getSubsystem(Shooter.class);
    this.indexer = RobotContainer.getSubsystem(Indexer.class);
    addRequirements(shooter, indexer);
  }

  /**
   * @param speed The speed, in m/s the flywheel moves at.
   * @param repeat If the command should be repeated on an interval.
   */
  public AutoShoot(double speed, boolean repeat) {
    this(speed, repeat, 13); // repeats every 260ms
  }

  /**
   * @param speed The speed, in m/s of the flywheel.
   */
  public AutoShoot(double speed) {
    this(speed, false, 13);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ballReady = false;
    framesTol = 0;
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(phase) {

      case ReadyToShoot:
        shooter.cmdVelocity(speed);

        count++;

        if(count == 20) {
          framesTol++;
          count = 0;
        }

        if(shooter.atSetpoint() || count >= framesTol) { // tolerance of 500ms
          phase = Phase.Shooting;
        }
        break;

      case Shooting:
        indexer.incrementPosition();
        phase = Phase.Finished;
        break;

      case Finished:
        indexer.cmdSetVelocity(0.0); // stop the indexer from spinning
        shooter.cmdVelocity(0.0);
        break;

      default:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.cmdSetVelocity(0.0);
    shooter.cmdVelocity(0.0);
    phase = Phase.ReadyToShoot;
    ballReady = false;
    framesTol = 0;
    count = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return phase == Phase.Finished;
  }
}