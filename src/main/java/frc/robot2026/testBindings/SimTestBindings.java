package frc.robot2026.testBindings;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.swerve.TargetCentricDrive;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.robot2026.command.shooter.AutoShoot;
import frc.robot2026.subsystems.Climber;
import frc.robot2026.subsystems.Hopper;
import frc.robot2026.subsystems.Intake;
import frc.robot2026.subsystems.Shooter.Indexer;
import frc.robot2026.subsystems.Shooter.Shooter;
import frc.robot2026.subsystems.Shooter.Targeter;

public class SimTestBindings {
    // subsystem references for use in command bindings
    public static DriveTrainInterface drivetrain;
    public static HID_Subsystem dc;
    public static Climber climber;
    public static Shooter shooter_left;
    public static Shooter shooter_right;
    public static Indexer indexer_left;
    public static Indexer indexer_right;
    public static Hopper hopper;
    public static Intake intake;
    public static Targeter targeter;

    private static void get_references() {
        // Subsystems must exist in RobotSpec, if they don't an NPE is thrown.
        shooter_left = RobotContainer.getSubsystem("shooter_left");
        shooter_right = RobotContainer.getSubsystem("shooter_right");
        drivetrain = RobotContainer.getSubsystem("drivetrain");  
        indexer_left = RobotContainer.getSubsystem("indexer_left");
        indexer_right = RobotContainer.getSubsystem("indexer_right");
        intake = RobotContainer.getSubsystem("intake");
        climber = RobotContainer.getSubsystem("climber");
        hopper = RobotContainer.getSubsystem(Hopper.class); 
        targeter = RobotContainer.getSubsystem(Targeter.class);
    }

    @SuppressWarnings("unused")
    public static void calbrate(CommandXboxController c) {
        get_references();
        c.x().whileTrue(new TargetCentricDrive(targeter).setP(4.0));
        c.leftTrigger(0.7).whileTrue(new AutoShoot("left", targeter::getTargetSpeedMotionCorrected, targeter::getTolerance, 1.0));
        c.leftTrigger(0.7)
                .whileTrue(new AutoShoot("right", targeter::getTargetSpeedMotionCorrected, targeter::getTolerance, 1.0));

    }
}

