// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANrange;
import frc.lib2202.command.WatcherCmd;
import frc.robot2026.Constants.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SensorsV2 extends SubsystemBase {
  /** Creates a new Sensors. */

  private CANrange canRange_L;
  private CANrange canRange_R;
  private Double canRange_L_Distance;
  private Double canRange_R_Distance;
  // Specify the CAN bus name, e.g., "rio" (roboRIO), "can0" (Linux), or a
  // CANivore name
  final CANBus kCANrangeCANbus = new CANBus("rio");

  public SensorsV2() {
    setName("sensorsV2");
    // Construct the CANrange object
    canRange_L = new CANrange(CAN.CANRANGE_L_CAN, kCANrangeCANbus);
    canRange_R = new CANrange(CAN.CANRANGE_R_CAN, kCANrangeCANbus);
    canRange_L_Distance = 0.0;
    canRange_R_Distance = 0.0;
    getWatcherCmd();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    canRange_L_Distance = canRange_L.getDistance().getValueAsDouble();
    canRange_R_Distance = canRange_R.getDistance().getValueAsDouble();
  }

  // Add a watcher so we can see stuff on network tables
  public WatcherCmd getWatcherCmd() {
    return this.new SensorV2Watcher();
  }

  public double getCANRangeL() {
    return canRange_L_Distance;
  }

  public double getCANRangeR() {
    return canRange_R_Distance;
  }

  class SensorV2Watcher extends WatcherCmd {
    SensorV2Watcher() {
        addEntry("canRange Left", SensorsV2.this::getCANRangeL);
        addEntry("canRange Right", SensorsV2.this::getCANRangeR);
      }
  }
}
