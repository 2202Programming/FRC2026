package frc.robot2026.subsystems.Shooter;

import static frc.lib2202.Constants.MperFT;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.util.PIDFController;
import frc.robot2026.Constants.CAN;
import frc.robot2026.subsystems.Shooter.FlyWheelRev.FlyWheelConfig;

public class Printer extends SubsystemBase {

    public Printer() {
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }

    @Override
    public void periodic() {
        // update hw, only needed if changes to HW_PID - TODO test mode?
    }

    // Basic Commands
    public Command printHello() {
        return runOnce(() -> {
            System.out.println("############ Hello#############");
        });
    }

   
    // Testing Bindings
    public void setTestBindings(CommandXboxController xbox) {
        xbox.leftTrigger(0.5)
                .onTrue(Commands.print("HELLO"));
        xbox.rightTrigger(0.5)
                .onTrue(printHello());
    }


}
