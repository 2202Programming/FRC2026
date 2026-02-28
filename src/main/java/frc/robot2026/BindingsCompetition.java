package frc.robot2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import frc.robot2026.subsystems.Climber;
import frc.robot2026.subsystems.Hopper;
import frc.robot2026.subsystems.Intake;
import frc.robot2026.subsystems.Shooter.Indexer;
import frc.robot2026.subsystems.Shooter.Shooter;
import frc.robot2026.subsystems.Shooter.Targeter;

/*
 * Please don't edit this without leads/mentor/driveteam review
 */
@SuppressWarnings("unused")
public final class BindingsCompetition {
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

        //Auto intake, works well if lightgate stays aligned.
        Trigger HasFuel = new Trigger(intake::hasFuel);
        HasFuel.onTrue(intake.cmdRunWhileFuel(.65, 0.5) );  //changed from .8 to 0.65 by drive team on 2/28

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

            // Shoot with targetSpeed based on distance to hub
            driver.leftTrigger(0.7).whileTrue(new AutoShoot(shooter_left, indexer_left, targeter::getTargetSpeed, 1));
            driver.leftTrigger(0.7).whileTrue(new AutoShoot(shooter_right, indexer_right, targeter::getTargetSpeed, 1));
            driver.leftTrigger(0.1).whileTrue(hopper.cmdBeltPct(1))
                    .onFalse(hopper.cmdBeltPct(0));

            // Driver wants to manually fire/pass
            driver.rightTrigger(0.7).whileTrue(new AutoShoot(shooter_left, indexer_left, targeter::getManualSpeed, 1));
            driver.rightTrigger(0.7)
                    .whileTrue(new AutoShoot(shooter_right, indexer_right, targeter::getManualSpeed, 1));
            driver.rightTrigger(0.1).whileTrue(hopper.cmdBeltPct(1))
                    .onFalse(hopper.cmdBeltPct(0));

        } else {
            DriverStation.reportError("Comp Bindings: No driver bindings set, check controllers.", false);
        }
    }

    static void OperatorBindings() {
        var sideboard = dc.SwitchBoard();
        var generic_opr = dc.Operator();

        Trigger Cal = sideboard.sw11(); // calibration button (conventional)
        Trigger NotCal = Cal.negate(); // regular competition mode
        Trigger DumbShooter = sideboard.sw26(); // placeholder for fallback to fixed shooting region

        // buttons depend on what controller is plugged in
        if (generic_opr instanceof CommandXboxController) {
            CommandXboxController operator = (CommandXboxController) generic_opr;
         
            // intake bindings
             //Testing this - DPL - auto run intake on lightgate
            // intake.FuelPresent.onTrue(intake.cmdRunForPeriod(.8, 2.0));       

            //intake in
            operator.leftBumper().whileTrue(intake.cmdPctPwr(0.65)) //changed from .8 on 2/27 by drive team
                                 .onFalse(intake.cmdPctPwr(0.0));
            // intake out
            operator.a().whileTrue(intake.cmdPctPwr(-0.80))
                                 .onFalse(intake.cmdPctPwr(0.0));

            // intake bindings

            // intake hopper in
            operator.leftBumper().whileTrue(hopper.cmdBeltPct(1))
                    .onFalse(hopper.cmdBeltPct(0));
            operator.leftBumper().whileTrue(intake.cmdPctPwr(0.80))
                    .onFalse(intake.cmdPctPwr(0.0));
            // intake and hopper out
            operator.rightBumper().whileTrue(hopper.cmdBeltPct(-1))
                    .onFalse(hopper.cmdBeltPct(0));
            operator.rightBumper().whileTrue(intake.cmdPctPwr(-0.80))
                    .onFalse(intake.cmdPctPwr(0.0));

            // shooter unblock
            operator.y().whileTrue(new AutoShoot(shooter_left, indexer_left, targeter::getUnblockSpeed, 1))
                    .whileTrue(new AutoShoot(shooter_right, indexer_right, targeter::getUnblockSpeed, 1));

            // agitate back and forth
            operator.a().whileTrue(new RepeatCommand(new SequentialCommandGroup(
                    hopper.cmdBeltPct(0.5),
                    new WaitCommand(.5),
                    hopper.cmdBeltPct(-0.5),
                    new WaitCommand(.5))))
                    .onFalse(hopper.cmdBeltPct(0));

            sideboard.sw14().onTrue(targeter.OverrideTargetDistanceFT(9.99)) // fixed distance
                    .onFalse(targeter.OverrideTargetDistanceFT(0.0)); // use vision distance

            // Calibration Commands
            Cal.and(sideboard.sw12()).whileTrue(climber.setVelocityCmd(Climber.ClimbCalibrateVel))
                    .onFalse(climber.setVelocityCmd(0.0));
            Cal.and(sideboard.sw13()).whileTrue(climber.setVelocityCmd(-Climber.ClimbCalibrateVel))
                    .onFalse(climber.setVelocityCmd(0.0));

            // climber arm extend to max
            operator.povUp().onTrue(climber.armsToPoint(Climber.ExtendPosition));

            // climber arm to whatever drive team sets
            operator.povDown().onTrue(climber.armsToPoint(Climber.RetractPosition));

            // manual flywheel speed adjustment
            operator.povLeft().onTrue(targeter.manualLow());
            operator.povRight().onTrue(targeter.manualHigh());

        } else {
            DriverStation.reportWarning("Comp Bindings: No operator bindings set, check controllers.", false);
        }

    }
}
