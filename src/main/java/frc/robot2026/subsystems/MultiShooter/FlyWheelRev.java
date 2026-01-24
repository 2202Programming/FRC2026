package frc.robot2026.subsystems.MultiShooter;

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
    public PIDFController hw_pid;       // just holds constants for pid, copyTo()/ copyChangesTo() updates hw
    //only sample values
    public double iMaxAccum=0.0;        // Integral of error[m/s]*[s] = [m]
    public double maxOpenLoopRPM=5600;  // [rpm] typical Neo
    public double gearRatio=1.0;        // [] [gearbox-rot/mtr-in] 
    public boolean inverted=true;      // adjust to spin in correct dir, account for motor/gears
    public double flywheelRadius = 0.05;// [m] ~2[in] in [m]
    public double rampRate=0.0;         // [s] time to max speed, 0 disables
    public int stallAmp=60;             // [Amp] stall current limit
    public int freeAmp=5;               // [Amp] current limit at free run
  };

  // Hardware
  final SparkMax controller; // this could be a generic controller controller...
  final SparkMaxConfig controllerCfg;
  final RelativeEncoder encoder;
  final SparkClosedLoopController closedLoopController;
  final ClosedLoopSlot kSlot = ClosedLoopSlot.kSlot0;
  final FlyWheelConfig cfg;
  final double posConverionFactor;

  // operational vars
  double vel_setpoint;
  double vel_tolerance = 0.5; // [m/s] flywheel edge linear velocity ~fuel speed

  public FlyWheelRev(int CAN_ID, FlyWheelConfig cfg) {
    this.cfg = cfg;
    controllerCfg = new SparkMaxConfig();
    controller = new SparkMax(CAN_ID, SparkMax.MotorType.kBrushless);
    posConverionFactor = 2.0 * Math.PI * cfg.flywheelRadius * cfg.gearRatio;
    // reset controller to factory
    controller.configure(controllerCfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    controllerCfg.inverted(cfg.inverted)
        .closedLoopRampRate(cfg.rampRate)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(cfg.stallAmp, cfg.freeAmp)
            // use physical used, m or m/s as seen at edge of flywheel, should be 1/2 fuel
            // cell velocity [m/s]
            .encoder // set driveEncoder to use units of the wheelDiameter, meters
        .positionConversionFactor(posConverionFactor) // [m]
        .velocityConversionFactor(posConverionFactor / 60.0); // [m/s]
    controllerCfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    // send values to hardware
    cfg.hw_pid.copyTo(controller, controllerCfg, kSlot);
    controller.configure(controllerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    closedLoopController = controller.getClosedLoopController();
    encoder = controller.getEncoder();

    setIMaxAccum(cfg.iMaxAccum);

    setSetpoint(0.0);
  }

  void copyChanges() {
    //writes to hw, does nothing if there are no pid related changes
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
  FlyWheelRev setSetpoint(double vel) { // setVelocitySetpoint
    vel_setpoint = vel;
    if (vel_setpoint == 0.0) {
      //force mode change to %pwr at zero, let it spin down, don't drive it to zero vel
      controller.set(vel_setpoint);
    }
    else {
      // closed loop velocity
      closedLoopController.setSetpoint(vel_setpoint, ControlType.kVelocity);
    }
    return this;
  }

  double getVelocity() {
    return encoder.getVelocity();
  }

  double getMotorRPM() {
    // return motor RPM by applying inverse velocity converstion factor
    return getVelocity() * 60.0 / posConverionFactor; 
  }

  double getTolerance() {
    return vel_tolerance;
  }

  double getPosition() {
    return encoder.getPosition();
  }

  double getPosRot() {
    return encoder.getPosition() / posConverionFactor;
  }

  double getIAccum() {
    return closedLoopController.getIAccum();
  }

  FlyWheelRev setVelocityTolerance(double vel_tolerance) {
    this.vel_tolerance = vel_tolerance;
    return this;
  }

  boolean atSetpoint() {
    return Math.abs(getVelocity() - vel_setpoint) <= vel_tolerance;
  }

  //expose ramprate so we can test its effect and tune to soften power spikes
  double getRampRate() {
    return cfg.rampRate;
  }
  
  FlyWheelRev setRampRate(double rate) {
    cfg.rampRate = rate;
    controllerCfg.closedLoopRampRate(rate);
    controller.configureAsync(controllerCfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    return this;
  }

  //expose iMaxAccum for tuning
  double getIMaxAccum(){
    return cfg.iMaxAccum;
  }

  FlyWheelRev setIMaxAccum(double iMaxAccum) {
    cfg.iMaxAccum = iMaxAccum;
    controllerCfg.closedLoop.iMaxAccum(iMaxAccum, kSlot);
    controller.configureAsync(controllerCfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    return this;
  }


}
