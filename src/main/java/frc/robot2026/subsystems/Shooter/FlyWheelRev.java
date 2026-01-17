package frc.robot2026.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.lib2202.util.PIDFController;

/**
 * Flywheel handles motor and gearing for the shooter flywheels.
 * 
 */
public class FlyWheelRev {

  public static class FlyWheelConfig {
    public PIDFController hw_pid; // just holds constants for pid, copyTo() updates hw
    public double maxOpenLoopRPM;
    public double gearRatio; // account for gearbox reduction to flywheel
    public boolean inverted;
    public double flywheelRadius;
    public int stallAmp;
    public int freeAmp;
  };

  // Hardware
  final SparkMax controller; // this could be a generic controller controller...
  final SparkMaxConfig controllerCfg;
  final RelativeEncoder encoder;
  final SparkClosedLoopController closedLoopController;
  final ClosedLoopSlot kSlot = ClosedLoopSlot.kSlot0;
  final FlyWheelConfig cfg;

  // operational vars
  double vel_setpoint;
  double vel_tolerance = 0.5; // [m/s] flywheel edge linear velocity ~fuel speed

  public FlyWheelRev(int CAN_ID, FlyWheelConfig cfg) {
    this.cfg = cfg;
    controllerCfg = new SparkMaxConfig();
    controller = new SparkMax(CAN_ID, SparkMax.MotorType.kBrushless);

    // reset controller to factory
    controller.configure(controllerCfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    controllerCfg.inverted(cfg.inverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(cfg.stallAmp, cfg.freeAmp)
            // use physical used, m or m/s as seen at edge of flywheel, should be 1/2 fuel
            // cell velocity [m/s]
            .encoder // set driveEncoder to use units of the wheelDiameter, meters
        .positionConversionFactor(2.0 * Math.PI * cfg.flywheelRadius / cfg.gearRatio) // [m]
        .velocityConversionFactor((2.0 * Math.PI * cfg.flywheelRadius / cfg.gearRatio) / 60.0); // [m/s]
    controllerCfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    // send values to hardware
    cfg.hw_pid.copyTo(controller, controllerCfg, kSlot);
    controller.configure(controllerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    closedLoopController = controller.getClosedLoopController();
    encoder = controller.getEncoder();

    setSetpoint(0.0);
  }

  void copyChanges() {
    cfg.hw_pid.copyChangesTo(controller, controllerCfg, kSlot);
  }

  // internal if we need to do something in subsys
  SparkBase getController() {
    return controller;
  }

  SparkClosedLoopController getClosedLoopController() {
    return closedLoopController;
  }

  // API used by sub-system
  void setSetpoint(double vel) { // setVelocitySetpoint
    vel_setpoint = vel;
    if (vel_setpoint == 0.0) {
      //force mode change to %pwr at zero, should let it keep spinning
      controller.set(vel_setpoint);
    }
    else {
      closedLoopController.setSetpoint(vel_setpoint, ControlType.kVelocity);
    }
  }

  double getVelocity() {
    return encoder.getVelocity();
  }

  double getTolerance() {
    return vel_tolerance;
  }

  FlyWheelRev setVelocityTolerance(double vel_tolerance) {
    this.vel_tolerance = vel_tolerance;
    return this;
  }

  boolean atSetpoint() {
    return Math.abs(getVelocity() - vel_setpoint) <= vel_tolerance;
  }

}