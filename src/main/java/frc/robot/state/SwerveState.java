// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.state;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Add your docs here. */
public class SwerveState {
    boolean swerveAutoBalance;
    boolean slowMode;
   public boolean swerveAutoAlign;

    private final JoystickButton toggleDriveTrainAutoBalance;
    private final JoystickButton slowButton;
    public SwerveState(Joystick driver){
        swerveAutoAlign = false;
        swerveAutoBalance = false;
        toggleDriveTrainAutoBalance =  new JoystickButton(driver, XboxController.Button.kStart.value);
        slowButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
        slowMode = false;

    }
    public void swerveAutoBalance(){
        swerveAutoBalance = true;
    }
    public void noSwerveAutoBalance(){
        swerveAutoBalance = false;
    }

    public void swerveAutoAlign(){
        swerveAutoAlign = true;
    }
    public void noSwerveAutoAlign(){
        swerveAutoAlign = false;
    }
    public boolean getSwerveState(){
        return swerveAutoBalance;
    }
    public void ToggleSlowMode(){
        slowMode = !slowMode;
    }
    public boolean getSlowMode(){
        return slowMode;
    }
    public boolean getSlowButton(){
        return slowButton.getAsBoolean();
    }
    public boolean getStartButton(){
        return toggleDriveTrainAutoBalance.getAsBoolean();
    }
}
