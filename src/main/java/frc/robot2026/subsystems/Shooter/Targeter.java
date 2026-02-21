package frc.robot2026.subsystems.Shooter;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
manage shooter speeds for different command use
    manual speed 
    ranged speed  
    distance to hub
    heading to  hub 
 */
public class Targeter extends SubsystemBase {
    final double HIGH_SPEED = 55.0;
    final double LOW_SPEED = 25.0;

    double distToTarget; // function of VPE pose and Hub center + math
    double manual_speed; // flywheel speed manually controlled by driver

    public Targeter() {

    }

    @Override
    public void initSendable(SendableBuilder builder) {

    }

    // cmd factory for use in complex commands
    // Basic Commands
    public Command manualHigh() {
        return runOnce(() -> {
            manual_speed = HIGH_SPEED;
        });

    }

    public Command manualLow() {
        return runOnce(() -> {
            manual_speed = LOW_SPEED;
        });

    }
}