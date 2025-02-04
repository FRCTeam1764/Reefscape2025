// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class IntakeRollers extends SubsystemBase {
  /** Creates a new IntakeRollers. */
  private TalonFX m_intakeMotor;
  private DigitalInput breakBeamIntake;

  public IntakeRollers() {
    m_intakeMotor = new TalonFX(Constants.INTAKE_MOTOR.id);


    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.Inverted =  InvertedValue.Clockwise_Positive;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeConfig.MotorOutput.PeakForwardDutyCycle = 0.8;
    intakeConfig.MotorOutput.PeakReverseDutyCycle = -0.8;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.StatorCurrentLimit = 60;

    
    m_intakeMotor.getConfigurator().apply(intakeConfig);

    breakBeamIntake = new DigitalInput(Constants.INTAKE_BREAK_BEAM);

  }

  double negative;
  public void wheelsIntake(double speed) {

    if (speed < 0) {
      negative = -1;
    } else {
      negative = 1;
    }

     if (!breakBeamIntake.get() && negative ==1) {
       m_intakeMotor.set(0);
     } else {
       m_intakeMotor.set(getPercentFromBattery(speed));
     }
     
  }
  public double getPercentFromBattery(double speed){
    return speed * 12 / RobotController.getBatteryVoltage();
  }
 
  public boolean getIntakeBreakbeam() {
    return !breakBeamIntake.get();
  }





  @Override
  public void periodic() {
    SmartDashboard.putBoolean("IntakeBreakbeam", getIntakeBreakbeam());
    SmartDashboard.putNumber("IntakeCurrent", m_intakeMotor.getMotorStallCurrent().getValueAsDouble());  }
}
