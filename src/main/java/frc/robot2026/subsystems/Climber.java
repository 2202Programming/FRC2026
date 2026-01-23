package frc.robot2026.subsystems;
// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.util.NeoServo;
import frc.lib2202.util.PIDFController;
import frc.robot2026.Constants.CAN;

public class Climber extends SubsystemBase {
    /** Creates a new Climber. */
    public final static double PowerUpPosition = 0.0; // [cm]
    public final static double ExtendPosition = 27.0; // [cm]
    public final static double ClimbPosition = -3.5; // [cm]
    public final static double ClimbCalibrateVel = 2.0; // [cm/s]

    final double GearRatio = 1.0 / 25.0;
    double conversionFactor = 3.5 * 2.54 * GearRatio;
    final double maxVel = 100.0; // placeholder. [cm/s]
    final double maxAccel = 10.0; // placevholder [cm/s^2]
    double posTol = 0.25; // [cm]
    double velTol = .50; // [cm/s]
    final int STALL_CURRENT = 60; // [Amp] placeholder 
    final int FREE_CURRENT = 5;   // [Amp] placeholder  
    
    public final Arm l_arm;
    public final Arm r_arm; 
    /* My logic for making these public is to allow access to the individual methods while outside the system.
       Otherwise that will just mean making more methods, and that seems like a waste - Gavin 
        Could be, normally SubSystem API should eliminate the need. We will see. Class needs to be public too 
        though to allow method access. - Mr.L
    */
    public class Arm implements Sendable {
        final NeoServo servo;
     
        // each arm needs own copy of pids, especially the softare position pid which is run by servo.periodic()
        PIDController posPID = new PIDController(4.0, 0.0015, 0.125);
        PIDFController hwVelPID = new PIDFController(0.02, 0.0, 0, 0.0285);

        Arm(int CANID, String side, boolean inverted) {            
            hwVelPID.setIZone(0.0);            
            hwVelPID.setIntegratorRange(0.0, 0.0);
            servo = new NeoServo(CANID, posPID, hwVelPID, inverted);
            setParams(CANID, side);            
        }

        private void setParams(int CANID, String side) {
            servo
                .setName("arm-"+side +"-" + CANID)
                .setConversionFactor(conversionFactor) // [cm]
                .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
                .setVelocityHW_PID(maxVel, maxAccel)
                .setTolerance(posTol, velTol)
                .setMaxVelocity(maxVel);
            servo.setPosition(PowerUpPosition);
        }
        
        public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty("vel_cmd",  this::getVelocityCmd, this::setVelocity );
            builder.addDoubleProperty("velocity",  this::getVelocity, null );
            builder.addDoubleProperty("vel_max", servo::getMaxVel, servo::setMaxVelocity);
        }

        //Arm API - mostly wrappers around servo
        public void setSetpoint(double pos) {            
            servo.setSetpoint(pos);  //goes into position mode
        }
        
        public double getSetpoint() {
            return servo.getSetpoint();
        }

        public double getPosition() {
            return servo.getPosition();
        }
                
        public void setPosition(double pos){
            servo.setPosition(pos);  //doesn't move, just tells servo here you are.
        }

        public double getVelocityCmd() {
            return servo.getVelocityCmd();
        }

        public void setVelocity(double vel) {
            servo.setVelocityCmd(vel);
        }

        public double getVelocity() {
            return servo.getVelocity();
        }

        public boolean atSetpoint() {
            return servo.atSetpoint();
        }        
    }

    public Climber() {
        // Set up in this format to use both arms as needed.
        l_arm = new Arm(CAN.l_arm,"L", true);
        r_arm = new Arm(CAN.r_arm,"R", true);
    }

    public Command setVelocityCmd(double vel, Arm arm) {
        return runOnce(() -> {
            arm.setVelocity(vel);  //switches Neo to vel mode
        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        l_arm.servo.periodic();
        r_arm.servo.periodic();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        l_arm.initSendable(builder);
        r_arm.initSendable(builder);        
    }
        
    // Climber API - most of these should use both l/r arms together I think
    // May not be necessary, arms will be running separately as they climb up the side of the building

    public Command setSetpointCmd(double pos, Arm arm) {
        return runOnce(() -> {
            arm.setSetpoint(pos);
        });         
    }

    public Command armsSetpointCmd(double pos) {
        return runOnce(() -> {  // simple instant cmd, sequenct not needed
            // if we need to move arms at same time, set both arms to same position
            l_arm.setSetpoint(pos);
            r_arm.setSetpoint(pos);
        });         
    }

    // set both arms to a given position, doesn't move, just initializes to position.
    // typical use at power up or after pitt calibration.
    public Command armsCalibrateCmd(double position) {
        return runOnce(() -> {
            //no sequence needed, these can run in single cmd.
            r_arm.setPosition(position);
            l_arm.setPosition(position);
        });
    }

    // TODO @Gavin,  add a command that waits until the arm is done moving to setpoint.  

    public boolean atSetpoint(){
        return l_arm.atSetpoint() && r_arm.atSetpoint();
    }



    public void setDemoBindings(CommandXboxController xbox) {
        /**
         * These are some basic test bindings for the climber, including a reset 0 position for when we do position testing.          
        */
        //velocity cmds while held it should spin, to test or align in pitt
        xbox.povLeft().whileTrue(this.setVelocityCmd(2.0, l_arm)).onFalse(this.setVelocityCmd(0.0, l_arm));
        xbox.povRight().whileTrue(this.setVelocityCmd(-2.0, l_arm)).onFalse(this.setVelocityCmd(0.0, l_arm));
        xbox.povUp().whileTrue(this.setVelocityCmd(2.0, r_arm)).onFalse(this.setVelocityCmd(0.0, r_arm));
        xbox.povDown().whileTrue(this.setVelocityCmd(-2.0, r_arm)).onFalse(this.setVelocityCmd(0.0, r_arm));

        // Move arms to 0 point
        xbox.x().onTrue(armsSetpointCmd(0.0)); 
        // tell the arms "here is zero"
        xbox.y().onTrue(armsCalibrateCmd(0.0));
    }
    
    class ClimberWatcher extends WatcherCmd {
        ClimberWatcher() {
            addEntry("L_position", Climber.this.l_arm::getPosition, 1);
            addEntry("R_position", Climber.this.r_arm::getPosition, 1);
            addEntry("AtSetpoint", Climber.this::atSetpoint);
            l_arm.servo.getWatcher();
            r_arm.servo.getWatcher();
        }
    }


}