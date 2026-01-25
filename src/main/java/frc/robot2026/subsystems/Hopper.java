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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.util.PIDFController;
import frc.robot2026.Constants.CAN;
import frc.robot2026.Constants.DigitalIO;

public class Hopper extends SubsystemBase {

  final SparkMax wideBeltCtrl;
  final SparkMax singleBeltCtrl;
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

  double P = 0.0;
  double I = 0.0;
  double D = 0.0;
  double F = 0.0;
  double kV = 12.0 / 5767.0; // Volts (somewhat arbitrary) / max RPM

  /**
   * Slot 0 is position control
   * Slot 1 is velocity control
   * 
   * Default slot is slot 0 --- must define a slot else it will default to slot 0
   */
  public Hopper() {
    setName("Hopper");

    wideBeltCtrl = new SparkMax(CAN.WideBeltID, MotorType.kBrushless);
    singleBeltCtrl = new SparkMax(CAN.SingleBeltID, MotorType.kBrushless);
    indexerCtrl = new SparkMax(CAN.IndexerID, MotorType.kBrushless);

    hwPidfCtrl = new PIDFController(P, I, D, F, "Indexer PIDF");

    indexGate = new DigitalInput(DigitalIO.HopperIndexerID);

    indexerEncoder = indexerCtrl.getEncoder();
    indexerCLCtrl = indexerCtrl.getClosedLoopController();
    indexerCfg = new SparkMaxConfig();

    indexerCfg.encoder
        .positionConversionFactor(posCF)
        .velocityConversionFactor(velCF);
    
    // SLOT 0 CONFIG - POSITION
    indexerCfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(P).i(I).d(D)
        .outputRange(-1, 1) // sets the range of the controller
        .feedForward
            .kV(kV, ClosedLoopSlot.kSlot0);
    
    // SLOT 1 CONFIG - VELOCITY
    indexerCfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(P).i(I).d(D)
        .outputRange(-1, 1) // sets the range of the controller
        .feedForward
            .kV(kV, ClosedLoopSlot.kSlot1);
    
    // POSITION CONTROL
    indexerCfg.closedLoop.maxMotion
        .cruiseVelocity(posCruiseVel, ClosedLoopSlot.kSlot0) 
        .maxAcceleration(posMaxAccel, ClosedLoopSlot.kSlot0);

    // VELOCITY CONTROL
    indexerCfg.closedLoop.maxMotion
        .cruiseVelocity(velCruiseVel, ClosedLoopSlot.kSlot1)
        .maxAcceleration(velMaxAccel, ClosedLoopSlot.kSlot1);

    indexerCtrl.configure(indexerCfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  // Expose PIDS and other values for tuning
  public void updatePosHardware() {
    hwPidfCtrl.copyChangesTo(indexerCtrl, indexerCfg, ClosedLoopSlot.kSlot0);
  }

  public void updateVelHardware() {
    hwPidfCtrl.copyChangesTo(indexerCtrl, indexerCfg, ClosedLoopSlot.kSlot1);
  }

  public double getIAccum() {
    return indexerCLCtrl.getIAccum();
  }

  public double getPosition() {
    return indexerEncoder.getPosition();
  }

  public double getVelocity() {
    return indexerEncoder.getVelocity();
  }
  
  // Vel + Pos control for the indexer
  public void setPosSetpoint(double pos) {
    indexerCLCtrl.setSetpoint(pos, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }

  public void setVelSetpoint(double vel) {
    indexerCLCtrl.setSetpoint(vel, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
  }
  
  // % Pwr control for wide belt + single belt
  public void setWideBeltPercent(double pct) {
    wideBeltCtrl.set(pct);
  }

  public void setSingleBeltPercent(double pct) {
    singleBeltCtrl.set(pct);
  }

  public void setBeltsPercent(double pct) {
    wideBeltCtrl.set(pct);
    singleBeltCtrl.set(pct);
  }

  // Commands to control belt pwr
  public Command cmdPct(double pct) {
    return run(() -> {
      setBeltsPercent(pct);
    });
  }

  public Command setSingleBeltPct(double pct) {
    return run(() -> {
      setSingleBeltPct(pct);
    });
  }

  public Command setWideBeltPct(double pct) {
    return run(() -> {
      setWideBeltPercent(pct);
    });
  }


  public Command setVelocity(double vel) {
    return run(() -> {
      setVelSetpoint(vel);
    });
  }

  public Command setPosition(double pos) {
    return run(() -> {
      setPosSetpoint(pos);
    });
  }

  public void setTestBindings(CommandXboxController xbox) {
    xbox.leftTrigger(0.5)
        .onTrue(cmdPct(0.5))
        .onFalse(cmdPct(0.0));
        
    xbox.rightTrigger(0.5)
        .onTrue(cmdPct(1.0))
        .onFalse(cmdPct(0.0));

    xbox.leftBumper()
        .onTrue(setVelocity(5.0))
        .onFalse(setVelocity(0.0));

    xbox.rightBumper()
        .onTrue(setPosition(5.0));
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("pct_pwr_wideBelt", this.wideBeltCtrl::get, this.wideBeltCtrl::set);
    builder.addDoubleProperty("pct_pwr_singleBelt", this.singleBeltCtrl::get, this.singleBeltCtrl::set);

    builder.addDoubleProperty("encoder_position", this.indexerEncoder::getPosition, this.indexerEncoder::setPosition);
    builder.addDoubleProperty("encoder_velocity", this.indexerEncoder::getVelocity, null);
  }



  public void periodic() {}
}