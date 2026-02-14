// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.wpilibj.RobotBase;
import frc.lib2202.builder.Robot;
import frc.robot2026.RobotSpec_ChassisBot;
import frc.robot2026.RobotSpec_ChassisBot_Finn;
import frc.robot2026.RobotSpec_BotOnBoard_Zeta;
import frc.robot2026.RobotSpec_AlphaBot;
import frc.robot2026.RobotSpec_BotOnBoard_Delta;
import frc.robot2026.RobotSpec_BotOnBoard_Epsilon;


public final class Main {
  private Main() {
    // create robot specs for supported robots in this binary
    //new RobotSpecDefault();  //example only, don't load spec

    // 2026 sub-tree
    new RobotSpec_ChassisBot();          // $env:serialnum = "03282B65"
    new RobotSpec_ChassisBot_Finn();     // $env:serialnum = "03415A8E"
    new RobotSpec_AlphaBot();            // $env:serialnum = "25AE07D"    
    new RobotSpec_BotOnBoard_Delta();    // $env:serialnum = "3061025"
    new RobotSpec_BotOnBoard_Epsilon();  // $env:serialnum = "0326F275"
    new RobotSpec_BotOnBoard_Zeta();     // $env:serialnum = "0312db1a"
    
  }
  public static void main(String... args) {
    new Main();
    RobotBase.startRobot(Robot::new);
  }
}   
