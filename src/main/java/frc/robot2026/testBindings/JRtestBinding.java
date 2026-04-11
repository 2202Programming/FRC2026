package frc.robot2026.testBindings;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.command.swerve.TargetCentricDrive;
import frc.robot2026.BindingsCompetition;
import frc.robot2026.command.shooter.AutoShoot;

public class JRtestBinding {

    // used to check rotation rates when calibrating wheel speeds
    public static void calbrate(CommandXboxController c) {
        c.rightBumper().onTrue(new TargetCentricDrive(BindingsCompetition.targeter));
        
        // Shoot with targetSpeed based on distance to hub
        c.leftTrigger(0.7).whileTrue(new AutoShoot("left", BindingsCompetition.targeter::getTargetSpeed, BindingsCompetition.targeter::getTolerance, 1.0));
        c.leftTrigger(0.7)
                .whileTrue(new AutoShoot("right", BindingsCompetition.targeter::getTargetSpeed, BindingsCompetition.targeter::getTolerance, 1.0));
        c.leftTrigger(0.1).whileTrue(BindingsCompetition.hopper.cmdBeltPct(0.705))
                .onFalse(BindingsCompetition.hopper.cmdBeltPct(0));
    }
}
