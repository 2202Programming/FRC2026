package frc.robot2026.subsystems;
// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    

    PIDController posPID = new PIDController(4.0, 0.0015, 0.125);
    PIDFController hwVelPID = new PIDFController(0.02, 0.0, 0, 0.0285);

    public Arm l_arm;
    public Arm r_arm; //My logic for making these public is to allow access to the individual methods while outside the system. Otherwise that will just mean making more methods, and that seems like a waste - Gavin 

    private class Arm {
        NeoServo servo;
        double desiredPos; // cm, 0 is full retract
        double desiredVel; // for network tables

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

        public double getDesiredPos() {
            return desiredPos;
        }

        public double getDesiredVel() {
            return desiredVel;
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

        public double getCurrent() {
            return servo.getController().getOutputCurrent();
        }

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

    public Command setVelocity(double vel, Arm arm) {
        return runOnce(() -> {
            arm.setArmVelocity(vel);  //switches Neo to vel mode
        });
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

    public void setDemoBindings(CommandXboxController xbox) {
        //bindings for Cycloid demo - use POV buttons with new ss cmd pattern        
        //velocity cmds while held it should spin
        xbox.povLeft().whileTrue(this.setVelocity(2.0, l_arm))
                      .onFalse(this.setVelocity(0.0, l_arm));

        xbox.povRight().whileTrue(this.setVelocity(-2.0, l_arm));
        
        // Cmd to known points
        xbox.povUp().whileTrue(this.setVelocity(2.0, r_arm)).onFalse(this.setVelocity(0.0, r_arm));
        xbox.povDown().whileTrue(this.setVelocity(-2.0, r_arm));
        xbox.y().onTrue(runOnce(() -> {l_arm.setClimberPos(0.0);
                                       r_arm.setClimberPos(0.0);
        })); //These are some basic test bindings for the climber, including a reset 0 position for when we do position testing. Yes this formating is stupid. No I don't care. -Gavin
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