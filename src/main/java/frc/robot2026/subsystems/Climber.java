package frc.robot2026.subsystems;
// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.util.NeoServo;
import frc.lib2202.util.PIDFController;
import frc.robot2026.Constants.CAN;

public class Climber extends SubsystemBase {

    /** Creates a new Climber. */
    public final static double StartPosition = 0.0; // [cm]
    public final static double ExtendPosition = 27.0; // [cm]
    public final static double ClimbPosition = -3.5; // [cm]
    public final static double ClimbCalibrateVel = 2.0; // [cm/s]

    final double GearRatio = 1.0 / 25.0;
    double conversionFactor = 3.5 * 2.54 * GearRatio;
    final double maxVel = 100.0; // placeholder. cm/s?
    final double maxAccel = 10.0; // placevholder cm/s^2
    double posTol = 0.25; // [cm]
    double velTol = .50; // [cm/s]
    final int STALL_CURRENT = 60; // placeholder // units?
    final int FREE_CURRENT = 30; // placeholder // units?
    double desiredPos; // cm, 0 is full retract
    double desiredVel;

    PIDController posPID = new PIDController(4.0, 0.0015, 0.125);
    PIDFController hwVelPID = new PIDFController(0.02, 0.0, 0, 0.0285);

    public Arm l_arm;
    public Arm r_arm; //My logic for making these public is to allow access to the individual methods while outside the system. Otherwise that will just mean making more methods, and that seems like a waste - Gavin 

    private class Arm {
        NeoServo servo;

        Arm(int CANID, boolean inverted) { 
                                                                     // 
            servo = new NeoServo(CANID, posPID, hwVelPID, inverted); //TODO Need to use 2 different PID controllers here?
        }

        public NeoServo getServo() {
            return servo;
        }

        public void setParams() {
            servo.setConversionFactor(conversionFactor) // in cm
                    .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
                    .setVelocityHW_PID(maxVel, maxAccel)
                    .setTolerance(posTol, velTol)
                    .setMaxVelocity(maxVel)
                    .burnFlash();
            servo.setPosition(StartPosition);
        }

        public void setSetpoint(double pos) {
            desiredPos = pos;
            servo.setSetpoint(pos);
        }

        public void setArmVelocity(double vel) {
            desiredVel = vel;
            servo.setVelocityCmd(vel);
        }

        public void setClimberPos(double pos) {
            servo.setPosition(pos);
        }

        public double getClimberPos() {
            return servo.getPosition();
        }

        public double getClimberVelocity() {
            return servo.getVelocity();
        }

        public boolean atSetpoint() {
            return servo.atSetpoint();
        }

        public void ClampVel(double vel) {
            MathUtil.clamp(vel, maxVel, -maxVel);
        }

        public void ClampAccel(double accel) {
            MathUtil.clamp(accel, maxAccel, -maxAccel);
        }

        // removed a get current command, may be important? double check with Mr L -Gavin
        public WatcherCmd getWatcherCmd() {
            return this.new ArmsWatcher();        
        }

        class ArmsWatcher extends WatcherCmd {
            ArmsWatcher() {            
                addEntry("position", Arm.this::getClimberPos);
                addEntry("velocity", Arm.this::getClimberVelocity, 2);
                addEntry("setpoint", ()-> {return servo.getSetpoint();});
            }       
        }
    }

    public Climber() {
        // Set up in this format to use both arms as needed.
        l_arm = new Arm(CAN.l_arm, true);
        r_arm = new Arm(CAN.r_arm, true);
        l_arm.setParams();
        r_arm.setParams();
        // should be done by NeoServo
        // //hwVelPID.copyTo(servo.getController().getClosedLoopController(),
        // ClosedLoopSlot.kSlot0);

    }

    /**
     * Extend arm to position given.
     *
     * @param pos Desired position in cm from fully retracted position
     */

    // lines 46-56 are for testing only
    /**
     * Testing command, sets the arms to a comanded velocity
     *
     */ 
    
    
    // void SetZero()

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        l_arm.getServo().periodic();
        r_arm.getServo().periodic();
    }

     
    // 
    // Someone smarter than me can probably get the watchers to work with how I have set this up, but these will work for now.
    //Lines 100-110 if you want to try and fix things -Gavin
    public WatcherCmd getWatcherCmdLeft() {
        return l_arm.getWatcherCmd();
    }

    public WatcherCmd getWatcherCmdRight() {
        return r_arm.getWatcherCmd();
    }

}