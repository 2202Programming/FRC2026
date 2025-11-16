package frc.robot2026;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib2202.builder.RobotContainer;

/*
 * Place commands named in PathPlaner autos here.
 */
@SuppressWarnings("unused")
public class RegisteredCommands {
    
    public static void RegisterCommands() {
        // NamedCommands.registerCommand("Pickup",   new InstantCommand(() -> {  elevator_Subsystem.setHeight(Levels.PickUp); }));
    }

}