package frc.robot2026;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib2202.builder.RobotContainer;
import frc.robot2026.command.pathing.runPath;

/*
 * Place commands named in PathPlaner autos here.
 */
@SuppressWarnings("unused")
public class RegisteredCommands {
    
    public static void RegisterCommands() {
    NamedCommands.registerCommand("shoot",   
        new PrintCommand("Shooting lots of fuel ... nothing but net."));

    NamedCommands.registerCommand("climb_right",   
        new PrintCommand("Climbing from right side."));

    NamedCommands.registerCommand("climb_left",   
        new PrintCommand("Climbing from left side."));

    }

}