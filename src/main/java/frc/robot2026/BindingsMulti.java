package frc.robot2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
import frc.robot2026.command.shooter.AutoShoot;
import frc.robot2026.command.shooter.AutoShootMulti;
import frc.robot2026.subsystems.Hopper;
import frc.robot2026.subsystems.Shooter.Indexer;
import frc.robot2026.subsystems.Shooter.Shooter;
import frc.robot2026.subsystems.Shooter.Targeter;

/*
 * Please don't edit this without leads/mentor/driveteam review
 */
@SuppressWarnings("unused")
public final class BindingsMulti {
    //subsystem references for use in command bindings
    public static DriveTrainInterface drivetrain; 
    public static HID_Subsystem dc;
    //public static Climber climber;
    public static Shooter shooter;
    public static Indexer indexerT;
    public static Indexer indexerB;
      //public static Hopper hopper;
    //public static MultiIntake intake;
    public static Targeter targeter;
    
    private static void get_references() {
        // Subsystems must exist in RobotSpec, if they don't an NPE is thrown.
        shooter = RobotContainer.getSubsystemOrNull("shooter");
        drivetrain = RobotContainer.getSubsystemOrNull("drivetrain");
        targeter = RobotContainer.getSubsystemOrNull("targeter");
        indexerT = RobotContainer.getSubsystemOrNull("indexer_top");
        indexerB = RobotContainer.getSubsystemOrNull("indexer_bottom");
        // hopper = RobotContainer.getSubsystem(Hopper.class);
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
            // Updated for m-roc per JB's request
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

            // LT indexer only
            operator.leftTrigger().whileTrue(indexerT.cmdSetPct(-0.1).alongWith(indexerB.cmdSetPct(-0.2)))
                                 .onFalse(indexerT.cmdSetPct(0).alongWith(indexerB.cmdSetPct(0))  );    
            // RT shoot normally                                 
            operator.rightTrigger(0.7).whileTrue(new AutoShootMulti(shooter, indexerT, indexerB, targeter::getManualSpeed, -0.5));

            // shooter unblock  - reverse flywheel
            operator.y().whileTrue(shooter.cmdVelocity(-15))
                        .onFalse(shooter.cmdVelocity(0.0));
            // pre-spin to last manual setting
            operator.b().whileTrue(shooter.cmdVelocity( targeter::getManualSpeed ))
                        .onFalse(shooter.cmdVelocity(0.0));

            // manual flywheel speed adjustment
            operator.povLeft().onTrue(targeter.manualLow());
            operator.povRight().onTrue(targeter.manualHigh());

        }
        else {
            DriverStation.reportWarning("Comp Bindings: No operator bindings set, check controllers.", false);
        }

    }
}