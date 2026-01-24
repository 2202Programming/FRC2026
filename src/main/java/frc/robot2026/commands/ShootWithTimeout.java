// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2026.subsystems.Shooter.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootWithTimeout extends Command {
  /** Creates a new Shoooter. */
  Shooter shooter;
  double shootSpeed; 
  Timer stopwatch; 
  double shooterTimeout=5;
  public ShootWithTimeout(double shootSpeed_r) {
    // Use addRequirements() here to declare subsystem dependencies.
     shooter =RobotContainer.getSubsystem(Shooter.class);
     //add feedder subsystems when ready 
     this.shootSpeed=shootSpeed_r;
  }
  public ShootWithTimeout(double shootSpeed_r,double shooterTimeout_r){
    this(shootSpeed_r);
    this.shooterTimeout=shooterTimeout_r;
  
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // spin up the fly wheel
    shooter.cmdVelocity(shootSpeed);
    stopwatch.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.atSetpoint()){
      //TODO: RUN FEEDDER 
      stopwatch.restart();
    }
    // if not at speed wait for next cycle
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.cmdVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (stopwatch.get() >shooterTimeout){
      return true;
    }else{
      return false;
    }
    
  }
}
