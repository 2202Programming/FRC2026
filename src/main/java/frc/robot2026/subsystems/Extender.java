package frc.robot2026.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot2026.Constants.CAN;

public class Extender extends SubsystemBase {
    final SparkFlex controller;
    final SparkFlexConfig config;
    final RelativeEncoder encoder;
    final SparkClosedLoopController closedLoopController;
    final FeedForwardConfig ffObj;

    final double GearRatio = 80.0;
    final double conversionFactor = 1.0 / GearRatio; // [rot (mtr) / rot (output)]

    final ClosedLoopSlot slot = ClosedLoopSlot.kSlot0;

    final boolean motor_inverted = true;

    final double cruiseVel = 0.5;
    final double maxAccel = .25;

    double P = 0.7;
    double I = 0.0015;
    double D = 0.0;
    double kS = 0.0;
    double kV = 12.0 / 5767.0;

    double pos_setpoint;
    boolean m_changes = false;

    boolean disable_servo = true;

    // Servo speed/positions
    double cmdPos;
    double cmdPct;

    public Extender() {
        setName("Extender-" + CAN.ExtendHopperID);
        controller = new SparkFlex(CAN.ExtendHopperID, MotorType.kBrushless);
        config = new SparkFlexConfig();
        encoder = controller.getEncoder();
        closedLoopController = controller.getClosedLoopController();
        config.closedLoop.maxMotion
                .cruiseVelocity(cruiseVel, slot)
                .maxAcceleration(maxAccel, slot)
                .allowedProfileError(1);
        ffObj = config.closedLoop.feedForward;

        config
                .inverted(motor_inverted);

        config.encoder
                .positionConversionFactor(conversionFactor);
        config.closedLoop
                .p(P, slot).i(I, slot).d(D, slot).feedForward
                .kS(kS, slot).kV(kV, slot);
        controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    public void setPosSetpoint(double setpoint) {
        closedLoopController.setSetpoint(setpoint, ControlType.kMAXMotionPositionControl, slot);
        pos_setpoint = setpoint;
    }

    public Command cmdSetPos(double setpoint) {
        return runOnce(() -> {
            setPosSetpoint(setpoint);
        });
    }

    public Command cmdZeroPos() {
        return runOnce(() -> {
            zeroPos();
        });
    }

    public Command cmdUpPos(){
        return runOnce(() ->{
            upPos();
        });
    }

    public void setkS(double newkS) {
        ffObj.kS(newkS);
        kS = newkS;
        m_changes = true;
    }

    public void setkV(double newkV) {
        ffObj.kV(newkV);
        kV = newkV;
        m_changes = true;
    }

    public double getPosSetPoint() {
        return pos_setpoint;
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public double getRPM() {
        return encoder.getVelocity();
    }

    public double getPositionError() {
        return Math.abs(getPosition() - pos_setpoint);
    }

    public void zeroPos() {
        encoder.setPosition(0.0);
    }
    public void upPos(){
        encoder.setPosition(1.0);
    }
    // velocity control only used for testing, normal cmds will use position
    public void setPercent(double pct) {
        cmdPct = pct;
        controller.set(pct);
    }

    public Command cmdPctPwr(double cmd_pct) {
        return Commands.runOnce(() -> {
            this.setPercent(cmd_pct);
        }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public void setTestBindings(CommandXboxController opr) {
        opr.leftBumper()
                .whileTrue(this.cmdPctPwr(0.5))
                .onFalse(this.cmdPctPwr(0.0));

        opr.rightBumper()
                .onTrue(this.cmdPctPwr(0.75))
                .onFalse(this.cmdPctPwr(0.0));

        opr.b()
                .onTrue(this.cmdPctPwr(0.0));

        opr.b().onTrue(new InstantCommand(() -> {
            zeroPos();
        }));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("pos_cmd", this::getPosSetPoint, this::setPosSetpoint);
        builder.addDoubleProperty("pos_err", this::getPosSetPoint, null);
        builder.addDoubleProperty("pos", this::getPosition, null);
        builder.addDoubleProperty("RPM", this::getRPM, null);

        builder.addDoubleProperty("kS", () -> {
            return kS;
        }, this::setkS);
        builder.addDoubleProperty("kV", () -> {
            return kV;
        }, this::setkV);

        builder.addDoubleProperty("P", () -> {
            return P;
        }, (double v) -> {
            this.P = v;
            m_changes = true;
        });

        builder.addDoubleProperty("I", () -> {
            return I;
        }, (double i) -> {
            this.I = i;
            m_changes = true;
        });

        builder.addDoubleProperty("D", () -> {
            return D;
        }, (double d) -> {
            this.D = d;
            m_changes = true;
        });
    }

}
