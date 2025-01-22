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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Constants;



public class Climbersubsystem extends SubsystemBase {
  /** Creates a new Climbersubsystem. */
  public TalonFX ClimberMotor;
  public DigitalInput limitSwitch;
  private PositionDutyCycle setVoltage;

  public Climbersubsystem() {
    TalonFX ClimberMotor = new TalonFX(Constants.CLIMBER_MOTOR.id);

    setVoltage = new PositionDutyCycle(null).withSlot(0);
    
    SetUpClimberMotors();
    
  }

  public void SetUpClimberMotors() {
    TalonFXConfiguration climbconfig = new TalonFXConfiguration();

    climbconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climbconfig.MotorOutput.PeakForwardDutyCycle = 1;
    climbconfig.MotorOutput.PeakReverseDutyCycle = -1;

    ClimberMotor.getConfigurator().apply(climbconfig);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0; 
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    ClimberMotor.getConfigurator().apply(slot0Configs);
  }
  
  public void climberOn(double desiredEncoderValue) {
    StatusCode val =   ClimberMotor.setControl(setVoltage.withPosition(desiredEncoderValue).withSlot(0));

  }

  public void zeroEncoder() {
    ClimberMotor.setPosition(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
