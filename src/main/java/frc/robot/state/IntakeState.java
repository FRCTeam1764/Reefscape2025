// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.state;

import frc.robot.constants.CommandConstants;

/** Add your docs here. */
public class IntakeState {
    double desired =CommandConstants.INTAKE_UP_ENCODERVALUE;
    
public void setEncoderValue(double value) {
    desired = value;
}
public double getEncoderValue() {
    return desired;
}
}