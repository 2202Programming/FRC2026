package frc.robot2026;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.command.swerve.calibrate.TestRotateVelocity;

public class DpltestBinding {

    public static void calbrate(CommandXboxController c) {
        c.rightBumper().onTrue(new TestRotateVelocity (90.0,4.0));
        c.leftBumper().onTrue(new TestRotateVelocity (30.0,6.0));
            
    }
}
