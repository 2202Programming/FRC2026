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
import frc.robot2026.subsystems.Shooter.FlyWheelRev.FlyWheelConfig;

public class Shooter extends SubsystemBase {
    final FlyWheelRev flywheel;
    final FlyWheelConfig cfg;

   //double vel_tolerance; // [m/s]

    public Shooter() {
        setName("Shooter-" + CAN.ShooterID);
        this.cfg = initFlyWheelConfig();
        flywheel = new FlyWheelRev(CAN.ShooterID, cfg);

        this.getWatcherCmd();
    }

    private FlyWheelConfig initFlyWheelConfig() {
        double kP = 0.04;       // tune next
        double kI = 0.00015;    // finally stiffen speed with I/D
        double kD = 3.0;
        double kF = 0.255;
        double iZone = 3.5;

        FlyWheelConfig cfg = new FlyWheelConfig();
        cfg.inverted = true;
        cfg.rampRate = 0.2;        // try to soften the startup, zero disables
        cfg.gearRatio = 0.6269;     // this was measured -- DPL + BG 1/19/26
        cfg.stallAmp = 40;          // [amp] Check motor specs for amps
        cfg.freeAmp = 5;            // [amp]
        cfg.maxOpenLoopRPM = 5800;  // measure at full power or motor spec
        cfg.flywheelRadius = (2.0 / 12.0) * MperFT; // [m] 2 [inch] converted [m]
        cfg.iMaxAccum = 2.5;
        // PIDF constant holder for hw
        cfg.hw_pid = new PIDFController(kP, kI, kD, kF, "flywheelPIDF");
        cfg.hw_pid.setIZone(iZone);
        return cfg;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("atVelocity", this::atSetpoint, null);
        builder.addDoubleProperty("iMaxAccum", flywheel::getIMaxAccum, flywheel::setIMaxAccum);
        builder.addDoubleProperty("iZone", cfg.hw_pid::getIZone, cfg.hw_pid::setIZone);
        builder.addDoubleProperty("ramp_rate", flywheel::getRampRate, flywheel::setRampRate);
        builder.addDoubleProperty("vel_cmd", ()->{ return flywheel.vel_setpoint;}, flywheel::setSetpoint);
        builder.addDoubleProperty("vel_measured", flywheel::getVelocity, null);
        builder.addDoubleProperty("vel_tolerance", flywheel::getTolerance, flywheel::setVelocityTolerance);
        // hook in the PID
        cfg.hw_pid.initSendable(builder);
    }

    @Override
    public void periodic() {
        // update hw, only needed if changes to HW_PID - TODO test mode?
        flywheel.copyChanges();
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
                .whileTrue(this.cmdVelocity(7.0)) // [m/s]
                .onFalse(this.cmdVelocity(0.0));
        xbox.rightTrigger(0.5)
                .whileTrue(this.cmdVelocity(14.0)) // [m/s]
                .onFalse(this.cmdVelocity(0.0));
        xbox.y().onTrue(new InstantCommand(() -> {
                this.flywheel.encoder.setPosition(0.0);
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
            addEntry("mtr_appliedOutput", Shooter.this.flywheel.getController()::getAppliedOutput, 2);
            addEntry("mtr_OutputAmps", Shooter.this.flywheel.getController()::getOutputCurrent, 2);
            addEntry("mtr_RPM", Shooter.this.flywheel::getMotorRPM, 1);
            addEntry("mtr_Temperature", Shooter.this.flywheel.getController()::getMotorTemperature, 2);
        }
    }

}
