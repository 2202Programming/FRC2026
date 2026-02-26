// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.command.WatcherCmd;
import frc.robot2026.Constants.CAN;

public class Intake extends SubsystemBase {

  final SparkFlex controller;
  final SparkFlexConfig config;
  final RelativeEncoder encoder;
  final SparkClosedLoopController closedLoopController;

  final double GearRatio = 5.0;
  final double conversionFactor = 1.0 / GearRatio; // [rot (mtr) / rot (output)]

  final ClosedLoopSlot slot = ClosedLoopSlot.kSlot0;

  final boolean motor_inverted = true;

  final double cruiseVel = 100.0;
  final double maxAccel = 75.0;

  double P = 0.0;
  double I = 0.0;
  double D = 0.0;
  double kS = 0.0;
  double kV = 12.0/5767.0;

  double pos_setpoint;

  boolean disable_servo = true;

  // Servo speed/positions
  double cmdPos;
  double cmdPct;

  /** Creates a new Intake. */
  public Intake() {
    setName("Intake-" + CAN.IntakeID);// + " | Intake-Bottom=" + CAN.IntakeBottomID);

    controller = new SparkFlex(CAN.IntakeID, MotorType.kBrushless);
    config = new SparkFlexConfig();
    encoder = controller.getEncoder();
    closedLoopController = controller.getClosedLoopController();

    config
        .inverted(motor_inverted);

    config.encoder
        .positionConversionFactor(conversionFactor);

    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    config.closedLoop.maxMotion
        .cruiseVelocity(cruiseVel, slot)
        .maxAcceleration(maxAccel, slot)
        .allowedProfileError(1);


    config.closedLoop
        .p(P).i(I).d(D)
    .feedForward
        .kS(kS).kV(kV);

    controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setPosSetpoint(double setpoint) {
      closedLoopController.setSetpoint(setpoint, ControlType.kMAXMotionPositionControl, slot);
      pos_setpoint = setpoint;
  }

  /*
   * getPosSetpoint()
   * getPosition()
   * getPosError()
   * zeroPos() - encoder
   * 
   * add set/get pos setpoint on the sendable
   * 
   * apply test bindings of zeroPos -> B
   */

  public double getPosSetPoint() {
    return pos_setpoint;
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public double getPositionError() {
    return Math.abs(getPosition() - pos_setpoint);
  }

  public void zeroPos() {
    encoder.setPosition(0.0);
  }

  // velocity control only used for testing, normal cmds will use position
  public void setPercent(double pct) {
    cmdPct = pct;
    controller.set(pct);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // power mode testing, disable servo if testing with duty-cycle
    if (!disable_servo) {
      // Roller.periodic();
    }
  }

  public Command cmdPctPwr(double cmd_pct) {
    return runOnce(() -> {
      this.setPercent(cmd_pct);
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

    opr.b().onTrue(new InstantCommand(() -> { zeroPos();} ));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("pos_cmd", this::getPosition, this::setPosSetpoint);
  }

  // Add a watcher so we can see stuff on network tables
  public WatcherCmd getWatcherCmd() {
    return this.new IntakeWatcher();
  }

  class IntakeWatcher extends WatcherCmd {
    IntakeWatcher() {

    }
  }
}