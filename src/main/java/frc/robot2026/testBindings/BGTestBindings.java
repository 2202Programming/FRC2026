package frc.robot2026.testBindings;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.RobotContainer;
import frc.robot2026.command.hopper.setPercent;
import frc.robot2026.command.shooter.AutoShootV2;
import frc.robot2026.subsystems.Hopper;
import frc.robot2026.subsystems.Shooter.Indexer;
import frc.robot2026.subsystems.Shooter.Shooter;

public class BGTestBindings {

    public static void calbrate(CommandXboxController c) {
        final Shooter shooterL = RobotContainer.getSubsystem("shooter_left");
        final Shooter shooterR = RobotContainer.getSubsystem("shooter_right");

        final Indexer indexerL = RobotContainer.getSubsystem("lIndexer");
        final Indexer indexerR = RobotContainer.getSubsystem("rIndexer");

        final Hopper hopper = RobotContainer.getSubsystem("hopper");

        if(shooterL == null || shooterR == null || indexerL == null || indexerR == null || hopper == null) {
            System.out.println("~~~~~~~~~~~~~SOMETHING IS NULL");
            return;
        }

        c.a().whileTrue(new AutoShootV2(shooterL, indexerL, 55.0, 1.0));
        c.a().whileTrue(new AutoShootV2(shooterR, indexerR, 55.0, 1.0));

        c.y().whileTrue(new AutoShootV2(shooterL, indexerL, 0.0, 0.0));
        c.y().whileTrue(new AutoShootV2(shooterR, indexerR, 0.0, 0.0));

        c.x().onTrue(new setPercent(hopper, 1.0));
        c.b().onTrue(new setPercent(hopper, 0.0));
    }
}
