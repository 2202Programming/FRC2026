package frc.robot2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.ScaleDriver;
import frc.lib2202.command.pathing.AllianceAwareGyroReset;
import frc.lib2202.command.swerve.RobotCentricDrive;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.hid.TMJoystickController;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;

import frc.robot2026.subsystems.Climber;
import frc.robot2026.subsystems.Intake;
import frc.robot2026.subsystems.Shooter.Shooter;

/*
 * Please don't edit this without leads/mentor/driveteam review
 */
@SuppressWarnings("unused")
public final class BindingsCompetition {
    //subsystem references for use in command bindings
    static DriveTrainInterface drivetrain; 
    static HID_Subsystem dc;
    static Climber climber;
    static Shooter shooter_left;
    static Shooter shooter_right;
    static Intake intake;
    
    private static void get_references() {
        // Subsystems must exist in RobotSpec, if they don't an NPE is thrown.
        climber = RobotContainer.getSubsystem(Climber.class);
        shooter_left = RobotContainer.getSubsystem("shooter_left");
        shooter_right = RobotContainer.getSubsystem("shooter_right");
        drivetrain = RobotContainer.getSubsystem("drivetrain");
        intake = RobotContainer.getSubsystem(Intake.class);
    }


    public static void ConfigureCompetition(HID_Subsystem dc) {       
        ConfigureCompetition(dc, true);
    }

    // optional disable opr binding for testing
    public static void ConfigureCompetition(HID_Subsystem _dc, boolean initOpr) {
         // get references for the commands to use
        dc = _dc;
        get_references();
        DriverBinding();
        if (initOpr) 
            OperatorBindings();
    }

    private static void DriverBinding() {       
        var generic_driver = dc.Driver();
        
        // Driver Buttons depend on the type of controller drivers selects
        if (generic_driver instanceof TMJoystickController) {
            // Joystick
            TMJoystickController joystick = (TMJoystickController) generic_driver;

        } else if (generic_driver instanceof CommandXboxController) {
            // XBox
            CommandXboxController driver = (CommandXboxController) generic_driver;
            driver.rightBumper().whileTrue(new RobotCentricDrive(drivetrain, dc));
            driver.y().onTrue(new AllianceAwareGyroReset());

            // Driver will wants precision robot-centric throttle drive on left bumper
            driver.leftBumper().whileTrue(new ParallelCommandGroup(
                    new ScaleDriver(0.3), 
                    new RobotCentricDrive(drivetrain, dc)));
                    
        } else {
            DriverStation.reportError("Comp Bindings: No driver bindings set, check controllers.", false);
        }
    }

    static void OperatorBindings() {
        var sideboard = dc.SwitchBoard();
        var generic_opr = dc.Operator();
       
        Trigger Cal = sideboard.sw11();  //calibration button (conventional)
        Trigger NotCal = Cal.negate();   // regular competition mode
        Trigger DumbShooter = sideboard.sw26();   // placeholder for fallback to fixed shooting region

        // buttons depend on what controller is plugged in
        if (generic_opr instanceof CommandXboxController) {
            CommandXboxController operator = (CommandXboxController) generic_opr;
         
            //Calibration Commands
            Cal.and(sideboard.sw21()).whileTrue(climber.setVelocityCmd(10.0, climber.l_arm))
                                     .onFalse(climber.setVelocityCmd(0.0,climber.l_arm));
            Cal.and(sideboard.sw22()).whileTrue(climber.setVelocityCmd(-10.0,climber.l_arm))
                                     .onFalse(climber.setVelocityCmd(0.0,climber.l_arm));

        }
        else {
            DriverStation.reportWarning("Comp Bindings: No operator bindings set, check controllers.", false);
        }

    }
}
