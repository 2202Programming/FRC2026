// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.subsystems.Shooter;

import java.time.format.ResolverStyle;

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

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot2026.Constants.CAN;
import frc.robot2026.Constants.DigitalIO;

public class Indexer extends SubsystemBase {

  final Roller leftRoller;
  final Roller rightRoller;

  double posCF = 1.0 / 9.0; // [ROT]
  double velCF = 1.0 / (9.0 * 60.0); // [RPS] of the INDEXER, not the MOTOR

  double cruiseVel = 5767.0;
  double maxAccel = 10000.0;

  double P = 0.3;
  double I = 0.0;
  double D = 0.0;

  double iMaxAccum = 0.015;
  double iZone = 20.0;

  double kV = 1.12; // Volts / max RPM
  double kS = 0.0; // amount of power required to overcome any mechanical slop and to make it barely move
  double kA = 0.0; // constant of acceleration - seems to increase acceleration

  public class Roller implements Sendable {
    final SparkFlex controller;
    final SparkFlexConfig controllerCfg;
    final RelativeEncoder encoder;
    final SparkClosedLoopController closedLoopController;
    final FeedForwardConfig ffObj;

    Roller(int CanID, boolean inverted, ClosedLoopSlot slot) {
      controller = new SparkFlex(CanID, MotorType.kBrushless);
      controllerCfg = new SparkFlexConfig();
      encoder = controller.getEncoder();
      closedLoopController = controller.getClosedLoopController();
      ffObj = controllerCfg.closedLoop.feedForward;
      configure(slot);
      configureTuning(slot);
    }

    private void configure(ClosedLoopSlot slot) {
      controllerCfg.encoder
        .positionConversionFactor(posCF)
        .velocityConversionFactor(velCF);

    indexerCtrl = new SparkFlex(CAN.LIndexerID, MotorType.kBrushless);

      controller.configure(controllerCfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void configureTuning(ClosedLoopSlot slot) {
      controllerCfg.closedLoop
        .p(P, slot).i(I, slot).d(D, slot) // PID
        .feedForward
        .kS(kS, slot).kV(kV, slot).kA(kA, slot); // SVA
    }

    public void setPosSetpoint()

    public void getPosSetPoint()

    

    public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("", null, null);
    }
  }

  final DigitalInput indexGate; // TODO: Unused currently

  // Used as a way to get and set new FF values
  final FeedForwardConfig ffObj;

  // Operational Variables
  double vel_setpoint;
  double pos_setpoint;
  double increment_position = 6.0; // rough estimate as of 2/20/2026
  boolean m_changes = false;

  /** Creates a new Indexer object */
  public Indexer() {
    setName("Hopper - " + CAN.LIndexerID);
    indexGate = new DigitalInput(DigitalIO.HopperIndexerID); // not being used as of 2/5/2026

    leftRoller = new Roller(CAN.LIndexerID, false, ClosedLoopSlot.kSlot0);
    rightRoller = new Roller(CAN.RIndexerID, false, ClosedLoopSlot.kSlot0);
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

  public Command incrementPosition() {
    return runOnce(() -> {
      setPosSetpoint(pos_setpoint + increment_position);
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

  public void setTestBindings(CommandXboxController xbox) {
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