// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Blinkin extends SubsystemBase {
  /** Creates a new Blinkin. */
  Double color;
  //preset colors:
  //-0.41  - blue ocean wave
  //-.99 - rainbow!!
  //.87 - blue 
  //-.09

  Spark blinkin  = new Spark(Constants.BLINKIN_SPARKPORT);
  public Blinkin() {
    color = -0.41;
  }

  public void setColor(double color){
this.color = color;
  }

  @Override
  public void periodic() {
    blinkin.set(color);
    // This method will be called once per scheduler run
  }
}
