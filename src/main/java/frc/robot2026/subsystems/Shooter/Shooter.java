package frc.robot2026.subsystems.Shooter;

import static frc.lib2202.Constants.MperFT;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.util.PIDFController;
import frc.robot2026.Constants.CAN;

public class Shooter extends SubsystemBase {
    final IFlyWheel flywheel;
    final FlyWheelConfig cfg;

    public Shooter() {
        this("rev");
    }

    public Shooter(String controllerType) {
        setName("Shooter-" + CAN.ShooterID);
        // pick which controller we are using
        if (controllerType.equalsIgnoreCase("ctre")) {
            cfg = initFlyWheelConfigCTRE();
            flywheel = new FlyWheelCtre(CAN.ShooterID, cfg);
        }
        else if (controllerType.equalsIgnoreCase("multi")) {
            cfg = initMultiFlyWheelConfigREV();
            flywheel = new FlyWheelRev(CAN.ShooterID, cfg);
        }
        else {
            cfg = initFlyWheelConfigREV();
            flywheel = new FlyWheelRev(CAN.ShooterID, cfg);
        }
        this.getWatcherCmd();        
    }

    //Setup using NEO1
    private FlyWheelConfig initFlyWheelConfigREV() {
        double kP = 0.005;       // tune next
        double kI = 0.00005;    // finally stiffen speed with I/D
        double kD = 10.0;       // Seems innsensitive until you add an extremely large value
        double kF = 0.65;
        double iZone = 1.0;     // setting it to 0.0 seems to 'unlock' it

        FlyWheelConfig cfg = new FlyWheelConfig();
        cfg.inverted = true;
        cfg.rampRate = 0.0;         // try to soften the startup, zero disables
        cfg.gearRatio = 0.6269;     // this was measured -- DPL + BG 1/19/26 
        cfg.stallAmp = 60;          // [amp] Check motor specs for amps
        cfg.freeAmp = 10;            // [amp]
        cfg.maxOpenLoopRPM = 5800;  // measure at full power or motor spec
        cfg.flywheelRadius = (2.0 / 12.0) * MperFT; // [m] 2 [inch] converted [m]
        cfg.iMaxAccum = 0.25;
        // PIDF constant holder for hw
        cfg.hw_pid = new PIDFController(kP, kI, kD, kF, "flywheelPIDF");
        cfg.hw_pid.setIZone(iZone);
        return cfg;
    }

    // tuning from MultiShooter, also rev Neo
    private FlyWheelConfig initMultiFlyWheelConfigREV() {
        double kP = 0.06;       // tune next
        double kI = 0.0001;    // finally stiffen speed with I/D
        double kD = 80;       // Seems innsensitive until you add an extremely large value
        double kF = 0.57;
        double iZone = 1.0;     // setting it to 0.0 seems to 'unlock' it

        FlyWheelConfig cfg = new FlyWheelConfig();
        cfg.inverted = false;
        cfg.rampRate = 0.0;         // try to soften the startup, zero disables
        cfg.gearRatio =  1.0;       // Richard changed as of 1/28/26 DPL/GL
        cfg.stallAmp = 80;          // [amp] Check motor specs for amps TESTING 80 FOR MULTI DUE TO HIGH DROP
        cfg.freeAmp = 10;            // [amp]
        cfg.maxOpenLoopRPM = 5800;  // measure at full power or motor spec
        cfg.flywheelRadius = (2.0 / 12.0) * MperFT; // [m] 2 [inch] converted [m]
        cfg.iMaxAccum = 0.25;
        // PIDF constant holder for hw
        cfg.hw_pid = new PIDFController(kP, kI, kD, kF, "flywheelPIDF");
        cfg.hw_pid.setIZone(iZone);
        return cfg;
    }

    // for testing Kraken
    private FlyWheelConfig initFlyWheelConfigCTRE() {
        double kP = 0.7;         // 
        double kI = 4.0;         // feels kind of bs
        double kD = 0.01;         // Seems innsensitive until you add an extremely large value
        double kF = 0.12;        // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V,
                                 //  1/8.33 =// 0.12 volts / rotation per second
        double iZone = 0.0;      // unused in Talon CTRE controller

        FlyWheelConfig cfg = new FlyWheelConfig();
        cfg.inverted = true;
        cfg.rampRate = 0.0;         // not implemented in ctre, but could be
        cfg.gearRatio = 18.0/24.0;  // new kraken pulleys
        cfg.stallAmp = 80;          // [amp] Use as stator amps
        cfg.freeAmp = 10;           // [amp] //unused
        cfg.maxOpenLoopRPM = 5800;  // measure at full power or motor spec
        cfg.flywheelRadius = (2.0 / 12.0) * MperFT; // [m] 2 [inch] converted [m]
        cfg.iMaxAccum = 0.0;        //unused in ctre
        // PIDF constant holder for hw
        cfg.hw_pid = new PIDFController(kP, kI, kD, kF, "flywheelPIDF");
        cfg.hw_pid.setIZone(iZone);
        return cfg;
    }



    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("atVelocity", this::atSetpoint, null);       
        builder.addDoubleProperty("vel_cmd", flywheel::getSetpoint, flywheel::setSetpoint);
        builder.addDoubleProperty("vel_measured", flywheel::getVelocity, null);
        builder.addDoubleProperty("vel_tolerance", flywheel::getTolerance, flywheel::setVelocityTolerance);

        // Rev Only
        if (flywheel instanceof FlyWheelRev) {
            var revfw = (FlyWheelRev)flywheel;
            builder.addDoubleProperty("iMaxAccum", revfw::getIMaxAccum, revfw::setIMaxAccum);
            builder.addDoubleProperty("iAccum", revfw::getIAccum, null);
            builder.addDoubleProperty("iZone", cfg.hw_pid::getIZone, cfg.hw_pid::setIZone);
            builder.addDoubleProperty("ramp_rate", revfw::getRampRate, revfw::setRampRate);
        }

        // hook in the PID
        cfg.hw_pid.initSendable(builder);
    }

    @Override
    public void periodic() {
        // update hw, only needed if changes to HW_PID - TODO test mode?
        flywheel.update_hardware();
    }

    // Add a watcher so we can see stuff on network tables
    public WatcherCmd getWatcherCmd() {
        return this.new ShooterWatcher();
    }

    // Shooter API
    public boolean atSetpoint() {
        return flywheel.atSetpoint();
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

    // Testing Bindings
    public void setTestBindings(CommandXboxController xbox) {
        xbox.leftTrigger(0.5)
                .whileTrue(this.cmdVelocity(10.0)) // [m/s]
                .onFalse(this.cmdVelocity(0.0));
        xbox.rightTrigger(0.5)
                .whileTrue(this.cmdVelocity(16.0)) // [m/s]
                .onFalse(this.cmdVelocity(0.0));
        xbox.b().onTrue(this.cmdVelocity(0.0)); // [m/s]
        xbox.y().onTrue(new InstantCommand(() -> {
                this.flywheel.setPosition(0.0);
            }));
    }

    // watcher will put values on the network tables for viewing elastic
    class ShooterWatcher extends WatcherCmd {
        ShooterWatcher() {
            addEntry("velocity", Shooter.this.flywheel::getVelocity, 2);
            addEntry("at_setpoint", Shooter.this::atSetpoint);
            addEntry("position", Shooter.this.flywheel::getPosition);
            addEntry("get_pos_rot", Shooter.this.flywheel::getPosRot);

            // other info about flywheel's motor
            addEntry("mtr_appliedOutput", Shooter.this.flywheel::getAppliedOutput, 2);
            addEntry("mtr_OutputAmps", Shooter.this.flywheel::getOutputCurrent, 2);
            addEntry("mtr_RPM", Shooter.this.flywheel::getMotorRPM, 1);
            addEntry("mtr_Temperature", Shooter.this.flywheel::getMotorTemperature, 2);
        }
    }
}
