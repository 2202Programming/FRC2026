// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.util.NeoServo;
import frc.lib2202.util.PIDFController;

public class Intake extends SubsystemBase {
  NeoServo bottomRoller;
  NeoServo topRoller;
  boolean disable_servo = false;

  PIDFController hwVelocity_PID = new PIDFController(0.0, 0.0, 0.0, 5.0 / 180.0 / 1.2); // [deg/s]
  PIDController  swPosition_PID = new PIDController(0.0 ,0, 0);  //[deg]

  //convert to deg/s units at the geared output
  final double GearRatio = 5.0;
  final double conversionFactor = 360.0 / GearRatio;  // [deg/rot]

  // Motor settings for Servo
  final int STALL_CURRENT = 80;
  final int FREE_CURRENT = 20;
  final boolean motor_inverted = true;
  // Servo speed/positions
  final double maxVel = 100.0;
  final double maxAccel = 75.0;

  double cmdPos;
  double cmdVel;

//   final SparkBase controller;
//   final SparkClosedLoopController cl_controller;

  /** Creates a new Intake. */
  public Intake() {
    // setup any other hardware Pid values, like Izone 
    hwVelocity_PID.setIZone(200.0); //[deg/s]  outside this region ignore integral

    bottomRoller = new NeoServo(50, swPosition_PID, hwVelocity_PID, motor_inverted);
    topRoller = new NeoServo(51, swPosition_PID, hwVelocity_PID, motor_inverted);

        // get the controllers out of the server so we can monitor in our watcher.
        // controller = servo.getController();
        // cl_controller = controller.getClosedLoopController();

        bottomRoller.setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT);
        topRoller.setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT);

        // We are going to use position command, so need to set power on POS
        // this also means some PIT trim needs to be done when shutting off for a match
  }

  // velocity control only used for testing, normal cmds will use position
  public void setPercent(double vel) {
    cmdVel = vel;
    bottomRoller.setVelocityCmd(vel);
    topRoller.setVelocityCmd(-vel);
  }

  public double getTVelocity() {
    return topRoller.getVelocity();
  }
  
  public double getBVelocity() {
    return bottomRoller.getVelocity();
  }

  // public double getMaxVel() {
  //   return servo.getMaxVel();
  // }

  // public void setMaxVelocity(double vel) {
  //   servo.setMaxVelocity(vel);
  // }

  // public double getCmdVelocity() {
  //   return cmdVel;
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // power mode testing, disable servo if testing with duty-cycle
    if (!disable_servo) {
      bottomRoller.periodic();
      topRoller.periodic();
    }
  }

  class ClimberWatcherCmd extends WatcherCmd {
    NetworkTableEntry nt_topVelocity;  
    NetworkTableEntry nt_btmVelocity;

    @Override
    public String getTableName() {
      return "Climber";
    }

    public void ntcreate() {
      NetworkTable table = getTable();
      nt_topVelocity = table.getEntry("topVelocity");
      nt_btmVelocity = table.getEntry("btmVelocity");
    }

    public void ntupdate() {
      nt_topVelocity.setDouble(getTVelocity());
      nt_btmVelocity.setDouble(getBVelocity());

    }
  }
}
