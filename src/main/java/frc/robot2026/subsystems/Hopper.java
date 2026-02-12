// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.util.PIDFController;
import frc.robot2026.Constants.CAN;
import frc.robot2026.Constants.DigitalIO;

public class Hopper extends SubsystemBase {

  // final SparkMax wideBeltCtrl;
  // final SparkMax singleBeltCtrl;
  final SparkMax indexerCtrl;
 
  final RelativeEncoder indexerEncoder;
  final SparkMaxConfig indexerCfg;
  final SparkClosedLoopController indexerCLCtrl;

  final PIDFController hwPidfCtrl;

  final DigitalInput indexGate;

  double posCF = 1.0; // temp
  double velCF = 1.0 / 60.0; // leaves in RPM

  double posCruiseVel =  5.0; //[RPS]
  double posMaxAccel = 5.0; //[RPS]

  double velCruiseVel = 5.0; //[RPS]
  double velMaxAccel = 5.0; //[RPS]

  final ClosedLoopSlot positionSlot = ClosedLoopSlot.kSlot0;
  final ClosedLoopSlot velocitySlot = ClosedLoopSlot.kSlot1;

  double P = 0.0;
  double I = 0.0;
  double D = 0.0;
  double F = 0.0;
    double kV = 12.0 / 5767.0; // Volts (somewhat arbitrary) / max RPM
    double kS = 0.15; // amount of power required to overcome any mechanical slop,
                      // and to start it barely moving.

  double vel_setpoint;
  double pos_setpoint;

  /**
   * Slot 0 is position control
   * Slot 1 is velocity control
   * 
   * Default slot is slot 0 --- must define a slot else it will default to slot 0
   */
  public Hopper() {
    setName("Hopper");

    // wideBeltCtrl = new SparkMax(CAN.WideBeltID, MotorType.kBrushless);
    // singleBeltCtrl = new SparkMax(CAN.SingleBeltID, MotorType.kBrushless);
    indexerCtrl = new SparkMax(CAN.IndexerID, MotorType.kBrushless);

    hwPidfCtrl = new PIDFController(P, I, D, F, "Indexer PIDF");

    indexGate = new DigitalInput(DigitalIO.HopperIndexerID); // not being used as of 2/5/2026

    indexerEncoder = indexerCtrl.getEncoder();
    indexerCLCtrl = indexerCtrl.getClosedLoopController();
    indexerCfg = new SparkMaxConfig();
    indexerCfg.encoder
        .positionConversionFactor(posCF)
        .velocityConversionFactor(velCF);
    
    // SLOT 0 CONFIG - POSITION
    indexerCfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(P, positionSlot).i(I, positionSlot).d(D, positionSlot) // incredibly hacky but it keeps all the PID stuffs in one place
        .feedForward
            .kV(kV, positionSlot)
            .kS(kS, positionSlot);
    
    // SLOT 1 CONFIG - VELOCITY
    indexerCfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(P, positionSlot).i(I, positionSlot).d(D, positionSlot) // incredibly hacky but it keeps all the PID stuffs in one place
        .feedForward
            .kV(kV, velocitySlot)
            .kS(kS, velocitySlot);
    
    // POSITION CONTROL
    indexerCfg.closedLoop.maxMotion
        .cruiseVelocity(posCruiseVel, positionSlot) 
        .maxAcceleration(posMaxAccel, positionSlot);

    // VELOCITY CONTROL
    indexerCfg.closedLoop.maxMotion
        .cruiseVelocity(velCruiseVel, velocitySlot)
        .maxAcceleration(velMaxAccel, velocitySlot);

    hwPidfCtrl.copyTo(indexerCtrl, indexerCfg, velocitySlot);
    
    indexerCtrl.configure(indexerCfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  // Add updating position and velocity hardware slots to the periodic
  // public void updatePosHardware() {
  //   hwPidfCtrl.copyChangesTo(indexerCtrl, indexerCfg, positionSlot);
  // }

  public void updateVelHardware() {
    hwPidfCtrl.copyChangesTo(indexerCtrl, indexerCfg, velocitySlot);
  }

  public void periodic() {
    updateVelHardware();
  }

  // *** POSITION ***
  public void setPosSetpoint(double pos) {
    return;
    // indexerCLCtrl.setSetpoint(pos, ControlType.kMAXMotionPositionControl, positionSlot);
  }

  public double getPosSetpoint() {
    return pos_setpoint;
  }

   public double getPosition() {
    return indexerEncoder.getPosition();
  }

  public void zeroPos() {
    indexerEncoder.setPosition(0.0);
  }

  // *** VELOCITY ***
  public void setVelSetpoint(double vel) {
    indexerCLCtrl.setSetpoint(vel, ControlType.kMAXMotionVelocityControl, velocitySlot);
    vel_setpoint = vel;
  }

  public double getVelSetpoint() {
    return vel_setpoint;
  }

  public double getVelocity() {
    return indexerEncoder.getVelocity();
  }

  // *** MISC PID VARS ***
  public double getIAccum() {
    return indexerCLCtrl.getIAccum();
  }

  // rampRate, iZone, iMaxAccum, kV, kS, free & stall amp, 
  
  // % Pwr control for wide belt + single belt
  // public void setWideBeltPercent(double pct) {
  //   wideBeltCtrl.set(pct);
  // }

  // public void setSingleBeltPercent(double pct) {
  //   singleBeltCtrl.set(pct);
  // }

  // public void setBeltsPercent(double pct) {
  //   wideBeltCtrl.set(-pct);
  //   singleBeltCtrl.set(pct);
  // }

  // Commands to control belt pwr
  // public Command cmdPct(double pct) {
  //   return runOnce(() -> {
  //     setBeltsPercent(pct);
  //   });
  // }

  // public Command setSingleBeltPct(double pct) {
  //   return runOnce(() -> {
  //     setSingleBeltPct(pct);
  //   });
  // }

  // public Command setWideBeltPct(double pct) {
  //   return runOnce(() -> {
  //     setWideBeltPercent(pct);
  //   });
  // }

  public Command cmdSetVelocity(double vel) {
    return runOnce(() -> {
      setVelSetpoint(vel);
    });
  }

  public Command setPosition(double pos) {
    return runOnce(() -> {
      setPosSetpoint(pos);
    });
  }

  public void setTestBindings(CommandXboxController xbox) {
    // xbox.leftTrigger(0.5)
    //     .onTrue(cmdPct(0.3))
    //     .onFalse(cmdPct(0.0));
        
    // xbox.rightTrigger(0.5)
    //     .onTrue(cmdPct(0.5))
    //     .onFalse(cmdPct(0.0));

    xbox.b().onTrue(new InstantCommand(() -> {
      this.zeroPos();
    }));

    xbox.a()
        .whileTrue(cmdSetVelocity(2.0))
        .onFalse(cmdSetVelocity(0.0));

    xbox.x()
        .whileTrue(cmdSetVelocity(5.0))
        .onFalse(cmdSetVelocity(0.0));

    xbox.y()
        .whileTrue(cmdSetVelocity(10.0))
        .onFalse(cmdSetVelocity(0.0));

    xbox.rightBumper().onTrue(cmdSetVelocity(0.0));
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    // builder.addDoubleProperty("pct_pwr_wideBelt", this.wideBeltCtrl::get, this.wideBeltCtrl::set);
    // builder.addDoubleProperty("pct_pwr_singleBelt", this.singleBeltCtrl::get, this.singleBeltCtrl::set);

    builder.addDoubleProperty("pos_cmd", this::getPosSetpoint, this::setPosSetpoint);
    builder.addDoubleProperty("vel_cmd", this::getVelSetpoint, this::setVelSetpoint);

    builder.addDoubleProperty("vel", this::getVelocity, null);
    builder.addDoubleProperty("pos", this::getPosition, null);

    // grab PIDs
    hwPidfCtrl.initSendable(builder);
  }
}