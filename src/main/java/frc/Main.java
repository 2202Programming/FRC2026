// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.wpilibj.RobotBase;
import frc.lib2202.builder.Robot;
import frc.robot2026.RobotSpec_ChassisBot;
import frc.robot2026.RobotSpec_BotOnBoard_Zeta;
import frc.robot2026.RobotSpec_BotOnBoard_Delta;
import frc.robot2026.RobotSpec_BotOnBoard_Epsilon;

public final class Main {
  private Main() {
    // create robot specs for supported robots in this binary
    //new RobotSpecDefault();  //example only, don't load spec

    // 2026 sub-tree
    new RobotSpec_ChassisBot(); 
    new RobotSpec_BotOnBoard_Zeta();
    new RobotSpec_BotOnBoard_Epsilon();
    new RobotSpec_BotOnBoard_Delta();

  }
  public static void main(String... args) {
    new Main();
    RobotBase.startRobot(Robot::new);
  }
}   
