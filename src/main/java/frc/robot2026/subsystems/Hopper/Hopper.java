// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.subsystems.Hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot2026.Constants.CAN;
import frc.robot2026.Constants.DigitalIO;

public class Hopper extends SubsystemBase {

  /*
   * Slot 0 is position control
   * Slot 1 is velocity control
   * 
   * The default slot is slot zero - you must define either
   * PositionSlot or VelocitySlot otherwise it will default
   * to slot 0 (PositionSlot).
   */
  final ClosedLoopSlot PositionSlot = ClosedLoopSlot.kSlot0;
  final ClosedLoopSlot VelocitySlot = ClosedLoopSlot.kSlot1;

  // Indexer SparkMAX requirements for MAXMotion
  final SparkFlex indexerCtrl;
  final SparkFlexConfig indexerCfg;
  final RelativeEncoder indexerEncoder;
  final SparkClosedLoopController indexerCLCtrl;

  // Wide/Single belt controllers are being controlled by % power
  final DigitalInput indexGate; // TODO: Unused currently
  // final SparkMax indexFeeder;
  // final SparkMax singleBeltCtrl;

  // Used as a way to get and set new FF values
  final FeedForwardConfig ffObj;

  double posCF = 1.0; // [ROT]
  double velCF = 1.0; // change to 1.0 / 60.0 for RPS

  double posCruiseVel = 5767.0;
  double posMaxAccel = 10000.0;

  double velCruiseVel = 5767.0; // Max RPM of the motor // [RPM]
  double velMaxAccel = 1000.0; // Max accel of the motor // [RPM/s]

  // These values are mostly dummy and will only work properly on a motor with no load
  double P = 0.0;
  double I = 0.0;
  double D = 0.0;

  double iMaxAccum = 0.015;
  double iZone = 20.0;

  double kV = 0.0; // Volts / max RPM
  double kS = 0.0; // amount of power required to overcome any mechanical slop and to make it barely move
  double kA = 0.0;

  double rampRate = 0.0; // untuned, unknown if needed

  // Operational Variables
  double vel_setpoint;
  double pos_setpoint;
  boolean m_changes = false;

  /** Creates a new Hopper object */
  public Hopper() {
    setName("Hopper - " + CAN.LIndexerID);

    indexerCtrl = new SparkFlex(CAN.LIndexerID, MotorType.kBrushless);
    // wideBeltCtrl = new SparkMax(CAN.WideBeltID, MotorType.kBrushless);
    // singleBeltCtrl = new SparkMax(CAN.SingleBeltID, MotorType.kBrushless);

    indexGate = new DigitalInput(DigitalIO.HopperIndexerID); // not being used as of 2/5/2026

    // construction of required MAXMotion pieces
    indexerEncoder = indexerCtrl.getEncoder();
    indexerCLCtrl = indexerCtrl.getClosedLoopController();
    indexerCfg = new SparkFlexConfig();
    indexerCfg.encoder
        .positionConversionFactor(posCF) // {1.0}
        .velocityConversionFactor(velCF); // {1.0}

    // *** SLOT 0 CONFIG - POSITION ***
    indexerCfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(P, PositionSlot).i(I, PositionSlot).d(D, PositionSlot)
        .feedForward
          .kV(kV, PositionSlot)
          .kS(kS, PositionSlot);

    indexerCfg.closedLoop.maxMotion
        .cruiseVelocity(posCruiseVel, PositionSlot)
        .maxAcceleration(posMaxAccel, PositionSlot)
        .allowedProfileError(2);

    // *** SLOT 1 CONFIG - VELOCITY ***
    indexerCfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(P, VelocitySlot).i(I, VelocitySlot).d(D, VelocitySlot)
        .feedForward
          .kV(kV, VelocitySlot)
          .kS(kS, VelocitySlot);

    indexerCfg.closedLoop.maxMotion
        .cruiseVelocity(velCruiseVel, VelocitySlot)
        .maxAcceleration(velMaxAccel, VelocitySlot);

    // used to set kV & kS
    ffObj = indexerCfg.closedLoop.feedForward;

    indexerCtrl.configure(indexerCfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * 
   * @param motorController - The motor controller to apply the values to
   * @param motorConfig     - The cfg to send the values into
   * @param slot            - The slot (PositionSlot / VelocitySlot) to send the
   *                        values to
   * 
   *                        Updates values every frame for PID, kV, kS, iMaxAccum,
   *                        iZone, rampRate, 
   */
  private void update(SparkBase motorController, SparkBaseConfig motorConfig, ClosedLoopSlot slot) {
    // skip if no changes or no attached hw typical if use PIDF without calling
    // copyTo()
    if (!m_changes || motorConfig == null || motorController == null) return;

    motorConfig.closedLoop.pid(P, I, D, slot);
    motorConfig.closedLoop.feedForward.sva(kS, kV, kA, slot);

    motorConfig.closedLoop.iMaxAccum(iMaxAccum, slot);
    motorConfig.closedLoop.iZone(iZone, slot);

    motorConfig.closedLoopRampRate(rampRate);

    motorConfig.closedLoop.maxMotion.maxAcceleration(velMaxAccel);

    // send to HW if we have a pid change, use async so robot loop isn't delayed
    var status = motorController.configureAsync(motorConfig, ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
    System.out.println(status); // hacky debug code
    m_changes = false;
  }

  @Override
  public void periodic() {
    // update(indexerCtrl, indexerCfg, VelocitySlot);
    update(indexerCtrl, indexerCfg, PositionSlot);
  }

  // *** POSITION GETTERS/SETTERS ***
  public void setPosSetpoint(double pos) {
    indexerCLCtrl.setSetpoint(pos, ControlType.kMAXMotionPositionControl, PositionSlot);
    pos_setpoint = pos;
  }

  public double getPosSetpoint() {
    return pos_setpoint;
  }

  public double getPosition() {
    return indexerEncoder.getPosition();
  }

  public double getPositionError() {
    return Math.abs(indexerCLCtrl.getMAXMotionSetpointPosition() - pos_setpoint);
  }

  // *** VELOCITY GETTERS/SETTERS ***
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

  // *** PID / kV & kS GETTERS & SETTERS ***
  // kV & kS
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

  public void setkA(double newkA) {
    ffObj.kA(newkA);
    kA = newkA;
    m_changes = true;
  }

  // PID
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

  // *** MISC SETTERS / GETTERS ***
  public double getRampRate() {
    
    return rampRate;
  }

  public void setRampRate(double newRampRate) {
    rampRate = newRampRate;
    m_changes = true;
  }

  public double getMaxAccel() {
    return velMaxAccel;
  }

  public void setMaxAccel(double newMaxAccel) {
    velMaxAccel = newMaxAccel;
    m_changes = true;
  }

  public double getAmps() {
    return indexerCtrl.getOutputCurrent();
  }

  // *** PERCENT POWER CONTROL ***
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

  public void setIdxPct(double pct) {
    indexerCtrl.set(pct);
  }

  // *** COMMANDS ***
  // MaxMotion
  public Command cmdSetVelocity(double vel) {
    return runOnce(() -> {
      setVelSetpoint(vel);
    });
  }

  public Command cmdSetPosition(double pos) {
    return runOnce(() -> {
      setPosSetpoint(pos);
    });
  }

  public Command cmdSetIdxPct(double pct) {
    return runOnce(() -> {
      setIdxPct(pct);
    });
  }

  public void zeroPos() {
    indexerEncoder.setPosition(0.0);
  }

  // Percent Power
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

  public void setTestBindings(CommandXboxController xbox) {
    // xbox.leftTrigger(0.5).onTrue(cmdPct(0.3)).onFalse(cmdPct(0.0));
    // xbox.rightTrigger(0.5).onTrue(cmdPct(0.5)).onFalse(cmdPct(0.0));

    xbox.a()
        .onTrue(cmdSetIdxPct(1.0)); // [ROT]

    xbox.x()
        .onTrue(cmdSetPosition(10.0)); // [ROT]

    xbox.y()
        .onTrue(cmdSetPosition(25.0)); // [ROT]

    xbox.b().onTrue(cmdSetPosition(0.0)); // [ROT]

    xbox.povDown().whileTrue(cmdSetVelocity(0.0)).onFalse(cmdSetVelocity(0.0));

    xbox.rightBumper().onTrue(new InstantCommand(() -> {
      zeroPos();
    }));
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

    // kS
    builder.addDoubleProperty("setkS", () -> { return this.kS; }, this::setkS);

    // kV
    builder.addDoubleProperty("setkV", () -> { return this.kV; }, this::setkV);

    // kA
    builder.addDoubleProperty("kA", () -> { return this.kA; }, this::setkA);

    // PID
    builder.addDoubleProperty("P", () -> { return P;}, (double v) -> {
      this.P = v;
      m_changes = true;
    });

    builder.addDoubleProperty("I", () -> { return I; }, (double i) -> {
      this.I = i;
      m_changes = true;
    });

    builder.addDoubleProperty("D", () -> { return D; }, (double d) -> {
      this.D = d;
      m_changes = true;
    });

    // iZone, iAccum, iMaxAccum
    builder.addDoubleProperty("iAccum", this::getIAccum, null);
    builder.addDoubleProperty("setIMaxAccum", () -> { return this.iMaxAccum; }, this::setIMaxAccum);
    builder.addDoubleProperty("iZone", () -> { return this.iZone; }, this::setIZone);

    // Miscellaneous
    builder.addDoubleProperty("rampRate", this::getRampRate, this::setRampRate);
    builder.addDoubleProperty("velocity acceleration", this::getMaxAccel, this::setMaxAccel);

    builder.addDoubleProperty("posError", this::getPositionError, null);

    builder.addDoubleProperty("amps", this::getAmps, null);
  }
}