package frc.robot2026.subsystems.Shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.OdometryInterface;

/*
manage shooter speeds for different command use
    manual speed 
    ranged speed  
    distance to hub
    heading to  hub 
 */
public class Targeter extends SubsystemBase {
    final OdometryInterface odo;
    Translation2d targetTranslation2d;

    InterpolatingTreeMap<Double, Double> rpm_table; // [M/S]
    double meas_dist;

    final double HIGH_SPEED = 55.0;
    final double LOW_SPEED = 25.0;

    double distToTarget; // function of VPE pose and Hub center + math
    double manual_speed; // flywheel speed manually controlled by driver

    public Targeter() {
        odo = RobotContainer.getSubsystem("odometry");

        InverseInterpolator<Double> distance = InverseInterpolator.forDouble();
        Interpolator<Double> rpm = Interpolator.forDouble();

        rpm_table = new InterpolatingTreeMap<>(distance, rpm);

        rpm_table.put(null, null);
        rpm_table.put(null, null);
        rpm_table.put(null, null);
        rpm_table.put(null, null);
        rpm_table.put(null, null);
    }

    // === INTERPOL GETTERS / SETTERS ===
    public double getRPMFromDistance(double distance) {
        return rpm_table.get(distance);
    }

    public void calculate() {
        meas_dist = odo.getDistanceToTranslation(targetTranslation2d);
    }

    public double getTargetRPM() {
        return getRPMFromDistance(meas_dist);
    }

    public double getTargetDistance() {
        return meas_dist;
    }
    
    public void setTarget() {
        var optAlliance = DriverStation.getAlliance(); // make sure this is accurate :)
        var alliance = optAlliance.isPresent() ? optAlliance.get() : DriverStation.Alliance.Blue;
    }

    // Basic Commands
    public Command manualHigh() {
        return runOnce(() -> {
            manual_speed = HIGH_SPEED;
        });

    }

    public Command manualLow() {
        return runOnce(() -> {
            manual_speed = LOW_SPEED;
        });
    }

    @Override
    public void initSendable(SendableBuilder builder) {

    }
}