package frc.robot2026.subsystems.Shooter;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
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
import frc.robot2026.Constants.TheField;

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
    
    // Provided by vince as angle between the center of the motor and the trailing edge of the ball exit ramp
    final double Shooter_Angle = 65.0; //Degrees

    Translation2d blueTarget;
    Translation2d redTarget;

    double distToTarget; // function of VPE pose and Hub center + math
    double manual_speed; // flywheel speed manually controlled by driver

    public Targeter() {

        Pose3d blue1 = TheField.fieldLayout.getTagPose(20).get();
        Pose3d blue2 = TheField.fieldLayout.getTagPose(26).get();
        blueTarget = new Translation2d(Math.abs(blue1.getX()-blue2.getX()) / 2.0, blue1.getY());

        Pose3d red1 = TheField.fieldLayout.getTagPose(4).get();
        Pose3d red2 = TheField.fieldLayout.getTagPose(10).get();
        redTarget = new Translation2d(Math.abs(red1.getX()-red2.getX()) / 2.0, red1.getY()); 

        odo = RobotContainer.getSubsystem("odometry");

        InverseInterpolator<Double> distance = InverseInterpolator.forDouble();
        Interpolator<Double> rpm = Interpolator.forDouble();

        rpm_table = new InterpolatingTreeMap<>(distance, rpm);
        
        // TODO These values will need to be tested for.
        // physics based pre-calculations:
        // https://docs.google.com/spreadsheets/d/1Cv1TSaGrY6Wx_dROy699KGdKHrFOiZqe/
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
        targetTranslation2d = (alliance == DriverStation.Alliance.Blue ? 
            blueTarget:redTarget);

        if (!optAlliance.isPresent())
            System.out.println("Warning: As no alliance was recived, we are defaulting to the Blue Alliance hub. This will go badly");
        // call calculate() to ensure update if target changed
        calculate();
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