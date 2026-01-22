package frc.robot2026;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static frc.lib2202.Constants.MperFT;
import static frc.lib2202.Constants.DEGperRAD;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib2202.builder.IRobotSpec;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.builder.SubsystemConfig;
import frc.lib2202.command.swerve.FieldCentricDrive;
import frc.lib2202.subsystem.Odometry;
import frc.lib2202.subsystem.OdometryInterface;
import frc.lib2202.subsystem.Sensors;
import frc.lib2202.subsystem.UX.TrimTables;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.swerve.AutoPPConfigure;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig.CornerID;
import frc.lib2202.util.PIDFController;
import frc.robot2026.Constants.CAN;
import frc.robot2026.subsystems.LimelightV2;
import frc.robot2026.subsystems.VisionPoseEstimator;

public class RobotSpec_BotOnBoard_Epsilon implements IRobotSpec {

  
  // Subsystems and other hardware on 2025 Robot rev Alpha
  // This should be the chassis bot.
  // $env:serialnum = "03282B65"
  final SubsystemConfig ssconfig = new SubsystemConfig("BOBEpsilon", "0326F275")
      // deferred construction via Supplier<Object> lambda
      .add(PowerDistribution.class, "PDP", () -> {
        var pdp = new PowerDistribution(CAN.PDP, ModuleType.kRev);
        pdp.clearStickyFaults();
        return pdp;
      });
  // Robot Speed Limits
  RobotLimits robotLimits = new RobotLimits(FeetPerSecond.of(15.0), DegreesPerSecond.of(180.0));

  public RobotSpec_BotOnBoard_Epsilon() {
    // add the specs to the ssconfig
    ssconfig.setRobotSpec(this);
  }

  // Required method that use the specs above

  @Override
  public RobotLimits getRobotLimits() {
    return robotLimits;
  }

  @Override
  public void setBindings() {
    //String odometryName = VisionPoseEstimator.class.getSimpleName(); // or novision "odometry"
    //TODO switch to vision based when we have a LL
    OdometryInterface odo = RobotContainer.getSubsystemOrNull("odometry");   
    DriveTrainInterface sdt = RobotContainer.getSubsystemOrNull("drivetrain");
    HID_Subsystem dc = RobotContainer.getSubsystemOrNull("DC");
    if (dc == null ){
        // BOB doesn't need DC. Return early if DC doesn't exist
        System.out.println("Warning: DC doesn't exist not setting bindings");
      return;
    }

    // Initialize PathPlanner, if we have needed Subsystems
    if (odo != null && sdt != null) {
      AutoPPConfigure.configureAutoBuilder(sdt, odo);
      var cmd = PathfindingCommand.warmupCommand();
      CommandScheduler.getInstance().schedule(cmd);
    }
    
    // Competition bindings 
    BindingsCompetition.ConfigureCompetition(dc, true);
    
    // Place your test binding in ./testBinding/<yourFile>.java and call it here
    // comment out any conflicting bindings. Try not to push with your bindings
    // active. Just comment them out. 
   

    // Anything else that needs to run after binding/commands are created
    /* 
    VisionPoseEstimator vpe = RobotContainer.getSubsystemOrNull(VisionPoseEstimator.class);
    if (vpe != null) 
      vpe.configureGyroCallback();
    */

    // show what cmds are running
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  SendableChooser<Command> autoChooser;

  @Override
  public void setupRegisteredCommands() {
    RegisteredCommands.RegisterCommands(); 

    //enable chooser - builds autochooser list, requires AutoBuilder to be configured
    //thus SDT and some form of odometry.  Skip auto if not configured.
    if (AutoBuilder.isConfigured()) {
      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", autoChooser);   
    }
  }
  
  @Override
  public SendableChooser<Command> getChooser() { 
    return autoChooser;
  }

  @Override
  public void setDefaultCommands() {
    DriveTrainInterface drivetrain = RobotContainer.getSubsystemOrNull("drivetrain");
    if (drivetrain != null) {
      drivetrain.setDefaultCommand(new FieldCentricDrive());
    }
  }

  

}