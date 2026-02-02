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

public class Intake extends SubsystemBase {
  final NeoServo bottomRoller;
  final NeoServo topRoller;

  final SparkBase btmRlrMtr;  // filled from Servo object
  final SparkBase topRlrMtr;  // filled from Servo object

  boolean disable_servo = false;

  PIDFController topHWVelocity_PID = new PIDFController(0.0, 0.0, 0.0, 5.0 / 180.0 / 1.2); // [deg/s]
  PIDFController bottomHWVelocity_PID = new PIDFController(0.0, 0.0, 0.0, 5.0 / 180.0 / 1.2); // [deg/s]
  PIDController  topPosition_PID = new PIDController(0.0 ,0, 0);  //[deg]
  PIDController  bottomPosition_PID = new PIDController(0.0 ,0, 0);  //[deg]

  //convert to deg/s units at the geared output
  final double GearRatio = 5.0;
  final double conversionFactor = 360.0 / GearRatio;  // [deg/rot]

  // Motor settings for Servo
  final int STALL_CURRENT = 80;
  final int FREE_CURRENT = 5;
  final boolean top_motor_inverted = true;
  final boolean bottom_motor_inverted = false;

  // Servo speed/positions
  final double maxVel = 100.0;
  final double maxAccel = 75.0;

  double cmdPos;
  double cmdPct;

//   final SparkBase controller;
//   final SparkClosedLoopController cl_controller;

  /** Creates a new Intake. */
  public Intake() {
    setName("Intake-Top=" + CAN.IntakeTopID + " | Intake-Bottom=" + CAN.IntakeBottomID);

    // setup any other hardware Pid values, like Izone 
    topHWVelocity_PID.setIZone(200.0); //[deg/s]  outside this region ignore integral
    bottomHWVelocity_PID.setIZone(200.0); //[deg/s]  outside this region ignore integral

    //Setup servos, for velocity or position control.
    topRoller = new NeoServo(CAN.IntakeTopID, topPosition_PID, topHWVelocity_PID, top_motor_inverted);
    bottomRoller = new NeoServo(CAN.IntakeBottomID, bottomPosition_PID, bottomHWVelocity_PID, bottom_motor_inverted);
    
    //Mr.L Feedback - can't recreate controllers with CANID, it was used by NeoServo, so pull from it
    btmRlrMtr = bottomRoller.getController(); //new SparkMax(50, SparkMax.MotorType.kBrushless);
    topRlrMtr = topRoller.getController();    // new SparkMax(51, SparkMax.MotorType.kBrushless);
        // get the controllers out of the server so we can monitor in our watcher.
        // controller = servo.getController();
        // cl_controller = controller.getClosedLoopController();

        bottomRoller.setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT);
        topRoller.setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT);

        // We are going to use position command, so need to set power on POS
        // this also means some PIT trim needs to be done when shutting off for a match
  }

  // velocity control only used for testing, normal cmds will use position
  public void setPercent(double pct) {
    cmdPct = pct;
    btmRlrMtr.set(pct);
    topRlrMtr.set(pct);
  }

  public double getTVelocity() {
    return topRoller.getVelocity();
  }
  
  public double getBVelocity() {
    return bottomRoller.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // power mode testing, disable servo if testing with duty-cycle
    if (!disable_servo) {
      bottomRoller.periodic();
      topRoller.periodic();
    }
  }

  public Command cmdPctPwr(double cmd_pct) {
    return run(() -> {
      this.setPercent(cmd_pct);
    });
  }

  public void setTestBindings(CommandXboxController opr) {
    opr.leftBumper()
        .whileTrue(this.cmdPctPwr(0.5))
        .onFalse(this.cmdPctPwr(0.0));

    opr.rightBumper()
        .whileTrue(this.cmdPctPwr(1.0))
        .onFalse(this.cmdPctPwr(0.0));
  }

  @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        //TODO add parameters here for tuning
        builder.addDoubleProperty("vel_1_top", this.topRoller::getVelocity, this.topRoller::setSetpoint);
        builder.addDoubleProperty("vel_2_bot", this.bottomRoller::getVelocity, this.bottomRoller::setSetpoint);

        builder.addDoubleProperty("pct_pwr_top", this.topRlrMtr::get, this.topRlrMtr::set);
        builder.addDoubleProperty("pct_pwr_btm", this.btmRlrMtr::get, this.btmRlrMtr::set);
    }

    // Add a watcher so we can see stuff on network tables
    public WatcherCmd getWatcherCmd() {
        return this.new IntakeWatcher();
    }


  class IntakeWatcher extends WatcherCmd {  
    IntakeWatcher() {
       addEntry("vel_top", Intake.this.topRoller::getVelocity, 2);
       addEntry("vel_bottom", Intake.this.bottomRoller::getVelocity, 2);
    }
  }

}

