package frc.robot2026;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /**
   * CAN bus IDs
   * 
   * Please keep in order ID order
   * 
   */
  public static final class CAN {
    public static final int ROBORIO = 0;
    public static final int PDP = 1; // for rev
    public static final int PCM1 = 2; // for rev
 
    // lights
    public static final int CANDLE1 = 3;
    public static final int CANDLE2 = 4;
    public static final int CANDLE3 = 5;
    public static final int CANDLE4 = 6;

    // SDT
    // synced as of 1/25/25 
    // https://docs.google.com/spreadsheets/d/1CyHzJscPIuvs0eFUY_qruQcuFui_w2nIXeXUyMwRKBU
    //
    public static final int BL_Angle = 20;
    public static final int BL_Drive = 21;
    public static final int BL_CANCoder = 28;

    public static final int FL_Angle = 23;
    public static final int FL_Drive = 22;
    public static final int FL_CANCoder = 29;

    public static final int BR_Angle = 25;
    public static final int BR_Drive = 24;
    public static final int BR_CANCoder = 31;

    public static final int FR_Angle = 26;
    public static final int FR_Drive = 27;
    public static final int FR_CANCoder = 30;

    // Intake
    public static final int IntakeTopID = 30;
    public static final int IntakeBotomID = 31;
   
    //Shooter
    public static final int ShooterID = 51;

    // IMU
    public static final int PIGEON_IMU_CAN = 60;
  }

  public static final class PWM{
    //public static final int Wrist = 0;
  }

  public static final class AnalogIn {
    //public static final int Wrist = 0;
  }

  // pnumatics control module 1
  public static final class PCM1 {
  }

  // pnumatics control module 2
  public static final class PCM2 {
  }

  public final class DigitalIO {
    //public static final int EndEffector_Lightgate = 2;
    //public static final int SignalLight1 = 7;
    //public static final int SignalLight2 = 8;
    //public static final int SignalLight3 = 9;              
  }

  //The Field info use WPILIB data
  public class TheField {
    //TODO update field for 2026
    public static AprilTagFields fieldChoice = AprilTagFields.k2025ReefscapeAndyMark; // k2025ReefscapeWelded;
    public static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(fieldChoice);
  }  

    public static class Vision {
      //photonvision camera names (needs to match photonvision UI naming)
      public static final String[] CAMERA_NAMES = {"HD_USB_Camera","USB_Camera"};

      // Robot to camera transforms.
      // Example: Cam mounted facing forward, half a meter forward of center, half a meter up from center.
      // Translation3d(0.5, 0.0, 0.5)
    
      public static final Transform3d[] kRobotToCam = {
              new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)),
              new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0))
            };


      // The layout of the AprilTags on the field
      public static final AprilTagFieldLayout kTagLayout =
              AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }
}