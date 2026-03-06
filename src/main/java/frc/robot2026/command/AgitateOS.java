package frc.robot2026.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.lib2202.builder.RobotContainer;
import frc.robot2026.subsystems.Hopper;
import frc.robot2026.subsystems.Intake;

public class AgitateOS extends Command {
    final double HopperSpeedDefault = 0.5;
    final Intake intake;
    final Hopper hopper;
    final Timer timer;
    final double period;
    final boolean hopperForward;
    final double intake_spd;        

    double in_spd, hop_spd;
    /**
     * AgitateOS - old school implementation, a work around for
     * use in a parallel cmd group in auto. See ncShoot()
     * 
     * @param hopperForward only run hopper forward
     * @param period        time to wait for switching dir
     * @param intake_spd    pct to drive intake 
     */
    public AgitateOS(boolean hopperForward, double period, double intake_spd) {
        this.intake = RobotContainer.getSubsystem("intake");
        this.hopper = RobotContainer.getSubsystem(Hopper.class);
        this.period = period;
        this.hopperForward = hopperForward;
        this.intake_spd = Math.abs(intake_spd);  // always start going in.
        this.timer = new Timer();
        addRequirements(intake, hopper);
    }
    
    // a few shortcut options
    public AgitateOS(double period, double intake_spd) {
        this(false, period, intake_spd);
    }
    
    public AgitateOS(double period) {
        this(false, period, 0.65);
    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        in_spd = intake_spd;
        hop_spd = HopperSpeedDefault;
        //set the hardware to start speeds
        intake.setPercent(in_spd);
        hopper.setBeltPct(hop_spd);
        timer.restart();
    }
    @Override
    public void execute() {
        // check timer for direction flip time
        if (timer.hasElapsed(period)) {
            in_spd = in_spd * -1.0;
            hop_spd = (hopperForward) ? HopperSpeedDefault : hop_spd * -1.0;
        
            //set the hardware to new speeds
            intake.setPercent(in_spd);
            hopper.setBeltPct(hop_spd);
            timer.restart();
        }
        // this is the part that was breaking the original implementation
        // needed to take over watching light gate, see getInterruptionBehavior().
        if (intake.hasFuel()) {
            //reset intake to going forward
            in_spd = intake_spd;
            intake.setPercent(in_spd);
        }
    }

    // stop everything when canceled
    @Override
    public void end(boolean interrupted) {
        intake.setPercent(0.0);
        hopper.setBeltPct(0.0);        
    }

    // this command NEVER finishes, it will be unscheduled, interrupted when done.
    @Override
    public boolean isFinished() {
        return false;
    }

    // we have to cancel incoming otherwise the Intake HasFuel trigger will
    // go off and cause us to end. This prevents it so this command can handle 
    // the intake's behavior completely.
    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }

}