package frc.robot2026.command.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot2026.subsystems.Shooter.Indexer;
import frc.robot2026.subsystems.Shooter.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoShootMulti extends Command {

  final Shooter shooter;
  final Indexer indexerT;
  final Indexer indexerB;

  DoubleSupplier speedProvider;
  double idxPct;
double cmdPower;

  public AutoShootMulti(Shooter shooter, Indexer indexerT, Indexer indexerB, DoubleSupplier speedProvider, double idxPct) {
    this.shooter = shooter;
    this.indexerT = indexerT;
    this.indexerB = indexerB;
    this.speedProvider = speedProvider;
    this.idxPct = idxPct;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmdPower = speedProvider.getAsDouble();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.flywheel.setSetpoint(cmdPower);
    
    if(shooter.atSetpoint()) {
      indexerT.setPct(idxPct);
      indexerB.setPct(idxPct);
    } else {
      indexerT.setPct(0.0);
      indexerB.setPct(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //shooter.flywheel.setSetpoint(0.0);
    indexerT.setPct(0.0);
    indexerB.setPct(0.0);
    // leave shooter running for 300ms after indexer is off
    // to make sure it is cleared
    double spd = speedProvider.getAsDouble();
    //create cmd and schedule it before we leave, 0.0 set at end
    var cmd = shooter.cmdVelocityDuration(spd, 0.300);
    CommandScheduler.getInstance().schedule(cmd);    
  }

  @Override
  public boolean isFinished() {
    return false;  //run until button is released
  }
}
    
