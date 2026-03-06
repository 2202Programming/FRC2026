package frc.robot2026;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.swerve.FaceToTag;
import frc.lib2202.command.swerve.RotateTo;
import frc.robot2026.command.Agitate;
import frc.robot2026.command.intake.Burp;
import frc.robot2026.command.pathing.runPath;
import frc.robot2026.command.shooter.AutoShoot;

/*
 * Place commands named in PathPlaner autos here.
 */
@SuppressWarnings("unused")
public class RegisteredCommands {

    // Named Command Factories

    public static Command ncShoot() {
        final double face_timeout = 1.0;   //pathing should leave use close
        final double shoot_timeout = 4.0;

        // grab the subsystem refs setup in bindings, to use for these cmds
        var shooter_left = BindingsCompetition.shooter_left;
        var shooter_right = BindingsCompetition.shooter_right;
        var indexer_left = BindingsCompetition.indexer_left;
        var indexer_right = BindingsCompetition.indexer_right;
        var targeter = BindingsCompetition.targeter;
        var hopper = BindingsCompetition.hopper;
        var intake = BindingsCompetition.intake;

        var cmd = new SequentialCommandGroup(
                new PrintCommand("Shooting lots of fuel ... nothing but net."),
                //FaceToTag() isn't great, better to use hub center and not tag
                //new FaceToTag(10, 26, face_timeout),
                // should rotate to hub center
                //new RotateTo(targeter.getRedHub(), targeter.getBlueHub(), face_timeout),
                // the commands in this parallel group DO NOT FINISH ...
                new ParallelCommandGroup(
                        new RepeatCommand(new Agitate(true)),
                        new AutoShoot("left", targeter::getTargetSpeed, targeter::getTolerance, 1),
                        new AutoShoot("right", targeter::getTargetSpeed, targeter::getTolerance, 1)
                ).withTimeout(shoot_timeout) // ... so we need this timeout.
        ).finallyDo(interrupted -> {
            hopper.setBeltPct(0.0);
            intake.setPercent(0.0); 
            System.out.println("ncShoot finalyDo lambda executed.");
        });

        cmd.addRequirements(shooter_left, shooter_right, indexer_left, indexer_right, hopper, intake);
        cmd.setName("ncShoot");
        return cmd;
    }

    public static void RegisterCommands() {
        // Construct all the commands and register them to NamedCommands for PathPlanner
        NamedCommands.registerCommand("shoot", ncShoot());

        NamedCommands.registerCommand("climb_right",
                new PrintCommand("Climbing from right side."));

        NamedCommands.registerCommand("climb_left",
                new PrintCommand("Climbing from left side."));

    }
}