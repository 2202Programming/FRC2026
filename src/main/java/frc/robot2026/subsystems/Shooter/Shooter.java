package frc.robot2026.subsystems.Shooter;

import static frc.lib2202.Constants.MperFT;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.util.PIDFController;
import frc.robot2026.Constants.CAN;
import frc.lib2202.subsystem.LightStack;
import frc.lib2202.subsystem.LightStack.LightRequest;

public class Shooter extends SubsystemBase {
    final public IFlyWheel flywheel;
    final FlyWheelConfig cfg;
    final boolean inverted;
    final String side; 
    int shots_taken=0;
    double speed_factor = 1.0;  // Simple factor to apply to requested speed, may be set in elastic
    LightRequest red = LightRequest.getRed();
    LightRequest green = LightRequest.getDefault().color(LightRequest.GREEN);

    public Shooter() {
        this("rev", 0, true);
    }

    public Shooter(String controllerType, int ShooterID) {
        this(controllerType, ShooterID, true);
    }

    public Shooter(String controllerType, int ShooterID, boolean inverted) {
        this.inverted = inverted;

        // Set side to left or right based on which shooter
        side = (ShooterID == CAN.ShooterIDLeft) ? "left" : "right";
        setName("Shooter_" + side);
        
        // Pick which controller we are using
        if (controllerType.equalsIgnoreCase("ctre")) {
            cfg = initFlyWheelConfigCTRE();

            flywheel = new FlyWheelCtre(ShooterID, cfg);
        } else if (controllerType.equalsIgnoreCase("multi")) {
            cfg = initMultiFlyWheelConfigREV();
            flywheel = new FlyWheelRev(ShooterID, cfg);
        } else if (controllerType.equalsIgnoreCase("flex_2")) {
            cfg = initFlyWheelConfigREVFlex2();
            flywheel = new FlyWheelRevFlex(ShooterID, cfg);
        } 
        else if (controllerType.equalsIgnoreCase("flex")) {
            cfg = initFlyWheelConfigREVFlex();
            flywheel = new FlyWheelRevFlex(ShooterID, cfg);
        } else {
            cfg = initFlyWheelConfigREV();
            flywheel = new FlyWheelRev(ShooterID, cfg);
        }

        
        
        int startID = (ShooterID == CAN.ShooterIDLeft) ? 3 : 22;
        red.range(startID, 1);
        green.range(startID, 1);
        //new BlinkyLights((ShooterID == CAN.ShooterIDLeft) ? CAN.CANDLE1 : CAN.CANDLE2);
        this.getWatcherCmd();
        
    }

    // Setup using NEO1
    private FlyWheelConfig initFlyWheelConfigREV() {
        double kP = 0.01;// 0.005; // tune next
        double kI = 0.00005; // Finally stiffen speed with I/D
        double kD = 2.0;// 10.0; // Seems innsensitive until you add an extremely large value
        double kF = 0.315;
        double iZone = 1.0; // Setting it to 0.0 seems to 'unlock' it

        FlyWheelConfig cfg = new FlyWheelConfig();
        cfg.inverted = inverted;
        cfg.rampRate = 0.0; // Try to soften the startup, zero disables
        cfg.gearRatio = 24.0 / 18.0; // This was measured -- DPL + BG 1/19/26
        cfg.stallAmp = 60; // [amp] Check motor specs for amps
        cfg.freeAmp = 10; // [amp]
        cfg.maxOpenLoopRPM = 5800.0; // Measure at full power or motor spec
        cfg.flywheelRadius = (2.0 / 12.0) * MperFT; // [m] 2 [inch] converted [m]
        cfg.iMaxAccum = 0.25;
        // PIDF constant holder for hw
        cfg.hw_pid = new PIDFController(kP, kI, kD, kF, "flywheelPIDF");
        cfg.hw_pid.setIZone(iZone);
        return cfg;
    }

    // Setup using Vortex
    private FlyWheelConfig initFlyWheelConfigREVFlex() { // RIGHT SHOOTER
        // Tuned by XS and AN on production alpha bot shooter
        double kP = 0.019;
        double kI = 0.0003;
        double kD = 7.0;
        double kF = 0.171;
        double iZone = 1.0; // Setting it to 0.0 seems to 'unlock' it

        FlyWheelConfig cfg = new FlyWheelConfig();
        cfg.inverted = inverted;
        cfg.rampRate = 0.0; // Try to soften the startup, zero disables
        cfg.gearRatio = 50.0 / 24.0; // mtr-side /fw-side
        cfg.stallAmp = 90; // [amp] Check motor specs for amps
        cfg.freeAmp = 15; // [amp]
        cfg.maxOpenLoopRPM = 5800.0; // Measure at full power or motor spec
        cfg.flywheelRadius = (2.0 / 12.0) * MperFT; // [m] 2 [inch] converted [m]
        cfg.iMaxAccum = 0.25;
        // PIDF constant holder for hw
        cfg.hw_pid = new PIDFController(kP, kI, kD, kF, "flywheelPIDF");
        cfg.hw_pid.setIZone(iZone);
        return cfg;
    }
 private FlyWheelConfig initFlyWheelConfigREVFlex2() { // LEFT SHOOTER
        // Tuned by XS and AN on production alpha bot shooter
        double kP = 0.05;
        double kI = 0.0003;
        double kD = 5.0;
        double kF = 0.323;
        double iZone = 1.0; // Setting it to 0.0 seems to 'unlock' it

        FlyWheelConfig cfg = new FlyWheelConfig();
        cfg.inverted = inverted;
        cfg.rampRate = 0.0; // Try to soften the startup, zero disables
        cfg.gearRatio = 40.0 / 38.0; // mtr-side /fw-side
        cfg.stallAmp = 90; // [amp] Check motor specs for amps
        cfg.freeAmp = 15; // [amp]
        cfg.maxOpenLoopRPM = 5800.0; // Measure at full power or motor spec
        cfg.flywheelRadius = (2.0 / 12.0) * MperFT; // [m] 2 [inch] converted [m]
        cfg.iMaxAccum = 0.25;
        // PIDF constant holder for hw
        cfg.hw_pid = new PIDFController(kP, kI, kD, kF, "flywheelPIDF");
        cfg.hw_pid.setIZone(iZone);
        return cfg;
    }



    // Tuning from MultiShooter, also rev Neo
    private FlyWheelConfig initMultiFlyWheelConfigREV() {
        double kP = 0.06; // Tune next
        double kI = 0.0001; // Finally stiffen speed with I/D
        double kD = 80; // Seems innsensitive until you add an extremely large value
        double kF = 0.57;
        double iZone = 1.0; // Setting it to 0.0 seems to 'unlock' it

        FlyWheelConfig cfg = new FlyWheelConfig();
        cfg.inverted = inverted;
        cfg.rampRate = 0.0; // Try to soften the startup, zero disables
        cfg.gearRatio = 1.0;
        cfg.stallAmp = 80; // [amp] Check motor specs for amps TESTING 80 FOR MULTI DUE TO HIGH DROP
        cfg.freeAmp = 10; // [amp]
        cfg.maxOpenLoopRPM = 5800.0; // Measure at full power or motor spec
        cfg.flywheelRadius = (2.0 / 12.0) * MperFT; // [m] 2 [inch] converted [m]
        cfg.iMaxAccum = 0.25;
        // PIDF constant holder for hw
        cfg.hw_pid = new PIDFController(kP, kI, kD, kF, "flywheelPIDF");
        cfg.hw_pid.setIZone(iZone);
        return cfg;
    }

    // For testing Kraken
    private FlyWheelConfig initFlyWheelConfigCTRE() {
        double kP = 0.7; //
        double kI = 4.0; // Feels kind of bs
        double kD = 0.01; // Seems innsensitive until you add an extremely large value
        double kF = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V,
                          // 1/8.33 =// 0.12 volts / rotation per second
        double iZone = 0.0; // Unused in Talon CTRE controller

        FlyWheelConfig cfg = new FlyWheelConfig();
        cfg.inverted = inverted;
        cfg.rampRate = 0.0; // Not implemented in ctre, but could be
        cfg.gearRatio = 1.0 / 1.0; // New kraken pulleys
        cfg.stallAmp = 80; // [amp] Use as stator amps
        cfg.freeAmp = 10; // [amp] // Unused
        cfg.maxOpenLoopRPM = 5800.0; // Measure at full power or motor spec
        cfg.flywheelRadius = (2.0 / 12.0) * MperFT; // [m] 2 [inch] converted [m]
        cfg.iMaxAccum = 0.0; // unused in ctre
        // PIDF constant holder for hw
        cfg.hw_pid = new PIDFController(kP, kI, kD, kF, "flywheelPIDF");
        cfg.hw_pid.setIZone(iZone);
        return cfg;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("SPEED_FACTOR", this::getSpeedFactor, this::setSpeedFactor);
        builder.addBooleanProperty("atVelocity", this::atSetpoint, null);
        builder.addDoubleProperty("vel_cmd", flywheel::getSetpoint, flywheel::setSetpoint);
        builder.addDoubleProperty("vel_measured", flywheel::getVelocity, null);
        builder.addDoubleProperty("vel_tolerance", flywheel::getTolerance, flywheel::setVelocityTolerance);

        // Rev Only
        if (flywheel instanceof FlyWheelRev) {
            var revfw = (FlyWheelRev) flywheel;
            builder.addDoubleProperty("iMaxAccum", revfw::getIMaxAccum, revfw::setIMaxAccum);
            builder.addDoubleProperty("iAccum", revfw::getIAccum, null);
            builder.addDoubleProperty("iZone", cfg.hw_pid::getIZone, cfg.hw_pid::setIZone);
            builder.addDoubleProperty("ramp_rate", revfw::getRampRate, revfw::setRampRate);
        }

        // Hook in the PID
        cfg.hw_pid.initSendable(builder);
    }

    @Override
    public void periodic() {
        // Update hw, only needed if changes to HW_PID - TODO test mode?
        flywheel.update_hardware();

        // Set lights to green if at setpoint, else set lights to red.
        if(atSetpoint())
        {
            LightStack.request(green);
        }
        else
        {
            LightStack.request(red);
        }
    }

    // Add a watcher so we can see stuff on network tables
    public WatcherCmd getWatcherCmd() {
        return this.new ShooterWatcher();
    }

    public void setSpeedFactor(double value) {
        this.speed_factor = value;
    }

    public double getSpeedFactor() {
        return this.speed_factor;
    }

    public int getShotsTaken() {
        return shots_taken;
    }

    public void addShots(int shots) {
        shots_taken += shots;
    }

    // Shooter API
    public boolean atSetpoint() {
        boolean off = flywheel.getSetpoint() == 0.0;
        return flywheel.atSetpoint() && !off;
    }

    // Basic Commands
    public Command cmdVelocity(double cmd_vel) {
        return runOnce(() -> {
            this.flywheel.setSetpoint(cmd_vel);
        });
    }

    public Command cmdVelocityWait(double cmd_vel) {
        return Commands.sequence(
                cmdVelocity(cmd_vel),
                Commands.waitUntil(this::atSetpoint),
                Commands.print(getName() + " is atSetpoint " + cmd_vel))
                .withName(getName() + ":cmdVelocityWait=" + cmd_vel);
    }

    //Use this to run the shooter for a short period of time to wind
    //down
    public Command cmdVelocityDuration(double cmd_vel, double seconds){
        return Commands.sequence(
                cmdVelocity(cmd_vel),
                new WaitCommand(seconds),
                cmdVelocity(0.0));
    }

    // Testing Bindings
    public void setTestBindings(CommandXboxController xbox) {
        xbox.leftTrigger(0.5)
                .whileTrue(this.cmdVelocity(65.0)) // [m/s]
                .onFalse(this.cmdVelocity(0.0));
        xbox.rightTrigger(0.5)
                .whileTrue(this.cmdVelocity(50.0)) // [m/s]
                .onFalse(this.cmdVelocity(0.0));
        xbox.leftBumper()
                .whileTrue(this.cmdVelocity(45.0)) // [m/s]
                .onFalse(this.cmdVelocity(0.0));
        xbox.rightBumper()
                .whileTrue(this.cmdVelocity(30.0)) // [m/s]
                .onFalse(this.cmdVelocity(0.0));

        xbox.b().onTrue(this.cmdVelocity(0.0)); // [m/s]
        xbox.y().onTrue(new InstantCommand(() -> {
            this.flywheel.setPosition(0.0);
        }));
    }

    // Watcher will put values on the network tables for viewing elastic
    class ShooterWatcher extends WatcherCmd {
        ShooterWatcher() {
            addEntry("_shots_", Shooter.this::getShotsTaken);
            addEntry("velocity", Shooter.this.flywheel::getVelocity, 2);
            addEntry("at_setpoint", Shooter.this::atSetpoint);
            addEntry("position", Shooter.this.flywheel::getPosition);
            addEntry("get_pos_rot", Shooter.this.flywheel::getPosRot);

            // Other info about flywheel's motor
            addEntry("mtr_appliedOutput", Shooter.this.flywheel::getAppliedOutput, 2);
            addEntry("mtr_OutputAmps", Shooter.this.flywheel::getOutputCurrent, 2);
            addEntry("mtr_RPM", Shooter.this.flywheel::getMotorRPM, 1);
            addEntry("mtr_Temperature", Shooter.this.flywheel::getMotorTemperature, 2);
        }
    }

    
}
