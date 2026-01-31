package frc.robot2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//add when needed - import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.pathing.AllianceAwareGyroReset;
import frc.lib2202.command.swerve.RobotCentricDrive;
import frc.lib2202.subsystem.OdometryInterface;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.hid.TMJoystickController;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.robot2026.subsystems.Climber;

/*
 * Please don't edit this without leads/mentor/driveteam review
 */
@SuppressWarnings("unused")
public final class BindingsCompetition {
   
    public static void ConfigureCompetition(HID_Subsystem dc) {
        ConfigureCompetition(dc, true);
    }

    // optional disable opr binding for testing
    public static void ConfigureCompetition(HID_Subsystem dc, boolean initOpr) {
        DriverBinding(dc);
        if (initOpr) OperatorBindings(dc);
    }

    private static void DriverBinding(HID_Subsystem dc) {
        OdometryInterface odo = RobotContainer.getObjectOrNull("odometry");
        DriveTrainInterface drivetrain = RobotContainer.getSubsystem("drivetrain");
        
        var generic_driver = dc.Driver();
        
        // Driver Buttons depend on the type of controller drivers selects
        if (generic_driver instanceof TMJoystickController) {
            // Joystick
            TMJoystickController joystick = (TMJoystickController) generic_driver;

            // put Driver's joystick bindings here

        } else if (generic_driver instanceof CommandXboxController) {
            // XBox
            CommandXboxController driver = (CommandXboxController) generic_driver;
            driver.rightBumper().whileTrue(new RobotCentricDrive(drivetrain, dc));
            driver.y().onTrue(new AllianceAwareGyroReset());

            // Driver will wants precision robot-centric throttle drive on left trigger
            driver.leftBumper().whileTrue(new ParallelCommandGroup(
                    //new ScaleDriver(0.3),  TODO add this cmd
                    new RobotCentricDrive(drivetrain, dc)));
                    
        } else {
            DriverStation.reportError("Comp Bindings: No driver bindings set, check controllers.", false);
        }
    }

    static void OperatorBindings(HID_Subsystem dc) {
        var sideboard = dc.SwitchBoard();
        var generic_opr = dc.Operator();
       
        // buttons depend on what controller is plugged in
        if (generic_opr instanceof CommandXboxController) {
            CommandXboxController operator = (CommandXboxController) generic_opr;

            Trigger Cal = sideboard.sw11();  //calibration button (conventional)
            Trigger NotCal = Cal.negate(); // regular competition mode

            Climber c = RobotContainer.getSubsystemOrNull(Climber.class);
            if (c != null) {
                c.setDemoBindings(operator);
                c.getWatcher();
            }
        }           
        else {
            DriverStation.reportWarning("Comp Bindings: No operator bindings set, check controllers.", false);
        }

        // Switchboard buttons too

        // Calibration commands

    }
}
