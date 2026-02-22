package frc.robot2026.subsystems.Shooter;

import static frc.lib2202.Constants.MperFT;

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

    InterpolatingTreeMap<Double, Double> vel_table; // [M/S]
    double meas_dist;

    final double HIGH_SPEED = 26.8; // [M/S]
    final double LOW_SPEED = 26.8; // [M/S]
    
    // Provided by vince as angle between the center of the motor and the trailing edge of the ball exit ramp
    final double Shooter_Angle = 65.0; // [deg]

    //Hub 
    final Translation2d blueHubTarget;
    final Translation2d redHubTarget;

    double distToTarget; // function of VPE pose and Hub center + math
    double manual_speed = LOW_SPEED; // flywheel speed manually controlled by driver

    public Targeter() {
        this("vision_odo");
    }
    
    public Targeter(String odo_name) {
        //use hub tags to calc center of blue and red hubs
        Pose3d blue1 = TheField.fieldLayout.getTagPose(20).get();
        Pose3d blue2 = TheField.fieldLayout.getTagPose(26).get();
        blueHubTarget = new Translation2d((blue1.getX()+blue2.getX()) / 2.0, blue1.getY());

        Pose3d red1 = TheField.fieldLayout.getTagPose(4).get();
        Pose3d red2 = TheField.fieldLayout.getTagPose(10).get();
        redHubTarget = new Translation2d((red1.getX()+red2.getX()) / 2.0, red1.getY()); 

        odo = RobotContainer.getSubsystem(odo_name);

        InverseInterpolator<Double> distance = InverseInterpolator.forDouble();
        Interpolator<Double> vel_mps = Interpolator.forDouble();

        vel_table = new InterpolatingTreeMap<>(distance, vel_mps);
        
        // Quick and dirty table measured on 2/21/26  
        // distance[m] -> flywheel [m/s]
        vel_table.put(0.0 * MperFT, 22.0);      // set a min
        vel_table.put(5.0 * MperFT, 22.0); 
        vel_table.put(6.0 * MperFT, 22.7);
        vel_table.put(10.0 * MperFT, 26.8);     // this is ladder radius
        vel_table.put(17.0 * MperFT, 32.5);
        vel_table.put(25.0 * MperFT, 32.5);     //set a max
    }
    public double getManualSpeed(){
        return manual_speed;
    }

    // === INTERPOL GETTERS / SETTERS ===
    public double getVelFromDistance(double distance) {
        return vel_table.get(distance);
    }

    public void calculate() {
        meas_dist = odo.getDistanceToTranslation(targetTranslation2d);
    }

    public double getTargetVelocity() {
        return getVelFromDistance(meas_dist);
    }

    public double getTargetDistance() {
        return meas_dist;
    }
    
    public void setTarget() {
        var optAlliance = DriverStation.getAlliance(); // make sure this is accurate :)
        var alliance = optAlliance.isPresent() ? optAlliance.get() : DriverStation.Alliance.Blue;
        targetTranslation2d = (alliance == DriverStation.Alliance.Blue ? 
            blueHubTarget : redHubTarget);

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
        builder.addDoubleProperty("target_dist", this::getTargetDistance, null);
    }
}