// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CommandConstants;
import frc.robot.constants.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  public TalonFX elevatorMotor1;
  public TalonFX elevatorMotor2;
  public DigitalInput limitSwitch;
  private PositionDutyCycle setVoltage;
  private PositionDutyCycle setVoltage2;
  private double desiredEncoder;
  
  int negative;
  public ElevatorSubsystem() {
    elevatorMotor1 = new TalonFX(Constants.ELEVATOR_MOTOR1.id, Constants.ELEVATOR_MOTOR1.busName);
    elevatorMotor2 = new TalonFX(Constants.ELEVATOR_MOTOR2.id, Constants.ELEVATOR_MOTOR2.busName);
    limitSwitch = new DigitalInput(Constants.ELEVATOR_SWITCH1);
    

    
    SetUpClimberMotors();
    
    setVoltage = new PositionDutyCycle(0).withSlot(0);
    setVoltage.UpdateFreqHz = 10;
  }

  public void SetUpClimberMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    TalonFXConfiguration config2 = new TalonFXConfiguration();

    config.Slot0.kP = 0.045; // p pid
    config.Slot0.kD = 0.00005; // d pid

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.PeakForwardDutyCycle = 1;
    config.MotorOutput.PeakReverseDutyCycle = -1; // can bump up to 12 or something
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 60; 
 
  
    config2.Slot0.kP = 0.435; // p pid
    config2.Slot0.kD = 0.00005; // d pid

    config2.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config2.MotorOutput.PeakForwardDutyCycle = 1;
    config2.MotorOutput.PeakReverseDutyCycle = -1; // can bump up to 12 or something
    config2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO: FIND IF TRUE OR NOT BEFORE U FRY ROBOT
    config2.CurrentLimits.StatorCurrentLimitEnable = true;
    config2.CurrentLimits.StatorCurrentLimit = 60; 

    elevatorMotor1.getConfigurator().apply(config);
    elevatorMotor2.getConfigurator().apply(config2);

    elevatorMotor2.setControl(new StrictFollower(elevatorMotor1.getDeviceID()));
  }
  
  public void elevatorOn(double desiredEncoderValue){
    desiredEncoder = desiredEncoderValue;
    //TOOD: NEED FEEDFORWARD
    StatusCode val =   elevatorMotor1.setControl(setVoltage.withPosition(desiredEncoderValue).withSlot(0));
  }

  public void elevatorOnSpeed(double speed){
    elevatorMotor1.set(speed);
  }

  public void off() {
    elevatorMotor1.set(0);
  }
  public double getDesiredEncoder(){
    return desiredEncoder;
  }
  public double getEncoderValue() {
    return elevatorMotor1.getPosition().getValueAsDouble();
  }

  public double getEncoderValue2() {
    return elevatorMotor2.getPosition().getValueAsDouble();
  }
  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }
  //TODO: find not safe encoder values
  public boolean Motor1IsSafe(){
    return Math.abs(getEncoderValue()) > 3 && Math.abs(getEncoderValue()) <200;
  }
  public boolean Motor2IsSafe(){
    return Math.abs(getEncoderValue()) > 3 && Math.abs(getEncoderValue()) <200;
  }
  
  public int retriveLevelEncoder(int level) {
    //if level == 1, return intake level one, else...
    return (level == 1) ? CommandConstants.ELEVATOR_LEVEL_ONE : 
    //if level == 2, return intake level two, else...
    (level == 2) ? CommandConstants.ELEVATOR_LEVEL_TWO :
    (level == 3) ? CommandConstants.ELEVATOR_LEVEL_THREE :
                   CommandConstants.ELEVATOR_LEVEL_FOUR;
  }

  public int retriveAngleEncoder(int level) {
    //if level == 1, return intake level one, else...
    return (level == 1) ? CommandConstants.WRIST_LEVEL_ONE : 
    //if level == 2, return intake level two, else...
    (level == 2) ? CommandConstants.WRIST_LEVEL_TWO :
    (level == 3) ? CommandConstants.WRIST_LEVEL_THREE :
                   CommandConstants.WRIST_LEVEL_FOUR;
  }

  public void zeroEncoder() {
    elevatorMotor1.setPosition(0);
    elevatorMotor2.setPosition(0);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("ElevatorMotor1Position", elevatorMotor1.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("ElevatorMotor2Position", elevatorMotor2.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("ElevatorMotor1Temperature", elevatorMotor1.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("ElevatorMotor2Temperature", elevatorMotor2.getDeviceTemp().getValueAsDouble());

    SmartDashboard.putNumber("ElevatorMotor1Current", elevatorMotor1.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("ElevatorMotor2Current", elevatorMotor2.getStatorCurrent().getValueAsDouble());

    SmartDashboard.putNumber("ElevatorMotor1Voltage", elevatorMotor1.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("ElevatorMotor2Voltage", elevatorMotor2.getMotorVoltage().getValueAsDouble());
  }
}
