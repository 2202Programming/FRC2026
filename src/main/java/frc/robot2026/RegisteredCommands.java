package frc.robot2026;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.swerve.FaceToTag;
import frc.robot2026.command.pathing.runPath;
import frc.robot2026.command.shooter.AutoShoot;

/*
 * Place commands named in PathPlaner autos here.
 */
@SuppressWarnings("unused")
public class RegisteredCommands {
   
    public static Command ncShoot() {
        //grab the subsystem refs setup in bindings, to use for these cmds
        var shooter_left = BindingsCompetition.shooter_left;
        var shooter_right = BindingsCompetition.shooter_right;
        var indexer_left = BindingsCompetition.indexer_left;
        var indexer_right = BindingsCompetition.indexer_right;
        var targeter = BindingsCompetition.targeter;
        var hopper = BindingsCompetition.hopper;
        var intake = BindingsCompetition.intake;

    // NamedCommands.registerCommand("shoot", 
    //     new ParallelCommandGroup(
    //         new PrintCommand("Shooting lots of fuel ... nothing but net."),
    //         new FaceToTag(10, 26), //wont work - old LL TODO FIX in lib2202
    //         new AutoShoot(shooter_left, indexer_left, targeter::getTargetSpeed, 1),
    //         new AutoShoot(shooter_right, indexer_right, targeter::getTargetSpeed, 1),
    //         hopper.cmdBeltPct(1)
    //         ).withTimeout(6.0)
    //          .withName("rc_shoot")
    //          .andThen(hopper.cmdBeltPct(0.0))
    //         );


    public static void RegisterCommands() {
       
    NamedCommands.registerCommand("shoot", ncShoot());       

    NamedCommands.registerCommand("climb_right",   
        new PrintCommand("Climbing from right side."));

    NamedCommands.registerCommand("climb_left",   
        new PrintCommand("Climbing from left side."));

    }

}