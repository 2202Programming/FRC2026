// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot2026.Constants.CAN;
import frc.robot2026.Constants.DigitalIO;

public class Hopper extends SubsystemBase {

   /**
   * Slot 0 is position control
   * Slot 1 is velocity control
   * 
   * Default slot is slot 0 --- must define a slot else it will default to slot 0 
   **/
  final ClosedLoopSlot PositionSlot = ClosedLoopSlot.kSlot0;
  final ClosedLoopSlot VelocitySlot = ClosedLoopSlot.kSlot1;

  // final SparkMax wideBeltCtrl;
  // final SparkMax singleBeltCtrl;
  final SparkMax indexerCtrl;
 
  final RelativeEncoder indexerEncoder;
  final SparkMaxConfig indexerCfg;
  final SparkClosedLoopController indexerCLCtrl;

  final FeedForwardConfig ffObj;

  final DigitalInput indexGate;

  double posCF = 1.0; // temp
  double velCF = 1.0; // leaves in RPM

  double posCruiseVel =  5.0; //[RPS]
  double posMaxAccel = 5.0; //[RPS]

  double velCruiseVel = 5767.0; //[RPM]
  double velMaxAccel = 4000.0; //[RPM]


  // These values are mostly dummy and will only work properly on a motor with no load
  double P = 0.00025;
  double I = 0.000000325;
    double iMaxAccum = 0.0125;
    double iZone = 20.0;
  double D = 0.00125;
  // double F = 0.0;
    double kV = 12.0 / 5767.0; // Volts (somewhat arbitrary) / max RPM
    double kS = 0.053; // amount of power required to overcome any mechanical slop,
                          // and to start it barely moving.

  double vel_setpoint;
  double pos_setpoint;

  boolean m_changes = false;
  
  public Hopper() {
    setName("Hopper");

    // wideBeltCtrl = new SparkMax(CAN.WideBeltID, MotorType.kBrushless);
    // singleBeltCtrl = new SparkMax(CAN.SingleBeltID, MotorType.kBrushless);
    indexerCtrl = new SparkMax(CAN.IndexerID, MotorType.kBrushless);

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
        .p(P, PositionSlot).i(I, PositionSlot).d(D, PositionSlot) // incredibly hacky but it keeps all the PID stuffs in one place
        .feedForward
            .kV(kV, PositionSlot)
            .kS(kS, PositionSlot);

    // SLOT 1 CONFIG - VELOCITY
    indexerCfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(P, VelocitySlot).i(I, VelocitySlot).d(D, VelocitySlot) // incredibly hacky but it keeps all the PID stuffs in one place
        .feedForward
            .kV(kV, VelocitySlot)
            .kS(kS, VelocitySlot);

     ffObj = indexerCfg.closedLoop.feedForward;
    
    // POSITION CONTROL
    indexerCfg.closedLoop.maxMotion
        .cruiseVelocity(posCruiseVel, PositionSlot) 
        .maxAcceleration(posMaxAccel, PositionSlot);

    // VELOCITY CONTROL
    indexerCfg.closedLoop.maxMotion
        .cruiseVelocity(velCruiseVel, VelocitySlot)
        .maxAcceleration(velMaxAccel, VelocitySlot);
    
    indexerCtrl.configure(indexerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Add updating position and velocity hardware slots to the periodic
  // public void updatePosHardware() {
  //   hwPidfCtrl.copyChangesTo(indexerCtrl, indexerCfg, positionSlot);
  // }

  private void update(SparkBase motorController, SparkBaseConfig motorConfig, ClosedLoopSlot slot) {
        // skip if no changes or no attached hw typical if use PIDF without calling copyTo()
        if (!m_changes || motorConfig == null || motorController == null) return;

        motorConfig.closedLoop.pid(P, I, D, slot);
        motorConfig.closedLoop.iMaxAccum(iMaxAccum, slot);
        motorConfig.closedLoop.iZone(iZone);
        motorConfig.closedLoop.feedForward.sv(kS, kV, slot);
        
        // send to HW if we have a pid change, use async so robot loop isn't delayed
        var status = motorController.configureAsync(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        System.out.println(status);
        m_changes = false;
    }

  @Override
  public void periodic() {
    update(indexerCtrl, indexerCfg, VelocitySlot);
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

  public void setkS(double newkS) {
    ffObj.kS(newkS);
    kS = newkS;
    m_changes = true;
  }

  public void setkV(double newkV) {
    ffObj.kV(newkV);
    kV = newkV;
    m_changes = true;
  }

  // *** VELOCITY ***
  public void setVelSetpoint(double vel) {
    indexerCLCtrl.setSetpoint(vel, ControlType.kMAXMotionVelocityControl, VelocitySlot);
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

  public double getIMaxAccum() {
    return iMaxAccum;
  }

  public void setIMaxAccum(double newIMaxAccum) {
    iMaxAccum = newIMaxAccum;
    m_changes = true;
  }

  public double getIZone() {
    return iZone;
  }

  public void setIZone(double newIZone) {
    iZone = newIZone;
    m_changes = true;
  }

  // rampRate, iZone, iMaxAccum, free & stall amp, 
  
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
        .whileTrue(cmdSetVelocity(100.0))
        .onFalse(cmdSetVelocity(0.0));

    xbox.x()
        .whileTrue(cmdSetVelocity(500.0))
        .onFalse(cmdSetVelocity(0.0));

    xbox.y()
        .whileTrue(cmdSetVelocity(1000.0))
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

    builder.addDoubleProperty("RPM", this::getVelocity, null);
    builder.addDoubleProperty("pos", this::getPosition, null);

    builder.addDoubleProperty("kS", () -> {return this.kS;}, null);
    builder.addDoubleProperty("setkS", null, this::setkS);
    builder.addDoubleProperty("kV", () -> {return this.kV;}, null);
    builder.addDoubleProperty("setkV", null, this::setkV);

    builder.addDoubleProperty("P", () -> {return P;}, (double v) -> {
                    this.P = v;
                    m_changes = true;
                  });
    
    builder.addDoubleProperty("I", () -> {return I;}, (double i) -> {
                    this.I = i;
                    m_changes = true;
                  });

        builder.addDoubleProperty("iAccum", this::getIAccum, null);
        builder.addDoubleProperty("setIMaxAccum", null, this::setIMaxAccum);
        builder.addDoubleProperty("iZone", null, this::setIZone);


    builder.addDoubleProperty("D", () -> {return D;}, (double d) -> {
                    this.D = d;
                    m_changes = true;
                  });
  }
}