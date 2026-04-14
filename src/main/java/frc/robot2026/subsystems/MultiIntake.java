// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.subsystems;

import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.util.NeoServo;
import frc.lib2202.util.PIDFController;
import frc.robot2026.Constants.CAN;

public class MultiIntake extends SubsystemBase {

  final NeoServo Roller;
  final NeoServo RollerUpper;

  final SparkBase Rlrmtr;  // filled from Servo object
  final SparkBase RlrmtrU;

  boolean disable_servo = true;

  PIDFController HWVelocity_PID = new PIDFController(0.0, 0.0, 0.0, 5.0 / 180.0 / 1.2); // [deg/s]
  PIDController  Position_PID = new PIDController(0.0 ,0, 0);  //[deg]

  //convert to deg/s units at the geared output
  final double GearRatio = 5.0;
  final double conversionFactor = 360.0 / GearRatio;  // [deg/rot]

  // Motor settings for Servo
  final int STALL_CURRENT = 80;
  final int FREE_CURRENT = 5;
  final boolean motor_inverted = true;


  // Servo speed/positions
  final double maxVel = 100.0;
  final double maxAccel = 75.0;

  double cmdPos;
  double cmdPct;

  /** Creates a new Intake. */
  public MultiIntake() {
    setName("MultiIntake-" + CAN.LIndexerMultiID + ":" + CAN.RIndexerMultiID);// + " | Intake-Bottom=" + CAN.IntakeBottomID);

    // setup any other hardware Pid values, like Izone 
    HWVelocity_PID.setIZone(200.0); //[deg/s]  outside this region ignore integral

    //Setup servos, for velocity or position control.
    Roller = new NeoServo(CAN.LIndexerMultiID, Position_PID, HWVelocity_PID, !motor_inverted);
    RollerUpper = new NeoServo(CAN.RIndexerMultiID, Position_PID, HWVelocity_PID, motor_inverted );
    
    //Mr.L Feedback - can't recreate contRollers with CANID, it was used by NeoServo, so pull from it
    Rlrmtr = Roller.getController();  
    RlrmtrU = RollerUpper.getController();

        Roller.setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT);
        RollerUpper.setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT);

  }

  // velocity control only used for testing, normal cmds will use position
  public void setPercent(double pct) { 
    setPercent(pct,pct); 
  
  }

  public void setPercent(double pct, double pct2) {
    cmdPct = pct;
    Rlrmtr.set(pct);
    RlrmtrU.set(pct2);
  }  

  public double getVelocity(NeoServo roller) {
    return roller.getVelocity();
  }

  public double getVelocity() {return getVelocity(Roller);}
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // power mode testing, disable servo if testing with duty-cycle
    if (!disable_servo) {
      Roller.periodic();
      RollerUpper.periodic();
    }
  }

  public Command cmdPctPwr(double cmd_pct) {
    return cmdPctPwr(cmd_pct,cmd_pct);}

  public Command cmdPctPwr(double cmd_pct, double pct_pwr_upper) {
    return runOnce(() -> {
      this.setPercent(cmd_pct,pct_pwr_upper);
    });
  }

  public void setTestBindings(CommandXboxController opr) {
    opr.leftBumper()
        .onTrue(this.cmdPctPwr(0.5))
        .onFalse(this.cmdPctPwr(0.0));

    opr.rightBumper()
        .onTrue(this.cmdPctPwr(0.75))
        .onFalse(this.cmdPctPwr(0.0));

    opr.b()
        .onTrue(this.cmdPctPwr(0.0));
  }

  @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        //TODO add parameters here for tuning
        builder.addDoubleProperty("vel_1", this.Roller::getVelocity, this.Roller::setSetpoint);

        builder.addDoubleProperty("pct_pwr", this.Rlrmtr::get, this.Rlrmtr::set);
    }

    // Add a watcher so we can see stuff on network tables
    public WatcherCmd getWatcherCmd() {
        return this.new IntakeWatcher();
    }


  class IntakeWatcher extends WatcherCmd {
    IntakeWatcher() {
       addEntry("vel", MultiIntake.this.Roller::getVelocity, 2);
    }
  }

}
