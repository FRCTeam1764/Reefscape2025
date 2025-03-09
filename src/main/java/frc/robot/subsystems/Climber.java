// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Constants;



public class Climber extends SubsystemBase {
  /** Creates a new Climbersubsystem. */
  public DigitalInput limitSwitch;
  private PositionDutyCycle setVoltage;
  public TalonFX m_climberMotor;

  public Climber() {
    m_climberMotor = new TalonFX(Constants.CLIMBER_MOTOR.id,Constants.CLIMBER_MOTOR.busName);

    setVoltage = new PositionDutyCycle(0).withSlot(0);
    
    SetUpClimberMotors();
    
  }

  public void SetUpClimberMotors() {
    TalonFXConfiguration climbconfig = new TalonFXConfiguration();

    climbconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climbconfig.MotorOutput.PeakForwardDutyCycle = 1;
    climbconfig.MotorOutput.PeakReverseDutyCycle = -1;
    climbconfig.CurrentLimits.StatorCurrentLimitEnable = true;
    climbconfig.CurrentLimits.StatorCurrentLimit = 60; 

    //m_climberMotor.getConfigurator().apply(climbconfig);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 2; 
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    m_climberMotor.getConfigurator().apply(slot0Configs);
  }

  public double getEncoderPos() {
    return m_climberMotor.getPosition().getValueAsDouble();
  }
  
  public void climberOn(double desiredEncoderValue) {
    

    m_climberMotor.set(desiredEncoderValue);
    //@SuppressWarnings("unused")

    //StatusCode val =   m_climberMotor.setControl(setVoltage.withPosition(desiredEncoderValue).withSlot(0));

  }

  public void zeroEncoder() {
    m_climberMotor.setPosition(0);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("ClimbMotorPosition", m_climberMotor.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("ClimbMotorTemperature", m_climberMotor.getDeviceTemp().getValueAsDouble());

    SmartDashboard.putNumber("ClimbMotorCurrent", m_climberMotor.getStatorCurrent().getValueAsDouble());

  }
}
