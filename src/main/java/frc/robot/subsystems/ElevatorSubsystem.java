// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  public TalonFX elevatorMotor1;
  public TalonFX elevatorMotor2;
  public PIDController pidController;
  public DigitalInput limitSwitch;
  private PositionDutyCycle setVoltage;
  private PositionDutyCycle setVoltage2;
  private double desiredEncoder;
  
  int negative;
  public ElevatorSubsystem() {
    elevatorMotor1 = new TalonFX(Constants.ELEVATOR_MOTOR1.id, Constants.ELEVATOR_MOTOR1.busName);
    elevatorMotor2 = new TalonFX(Constants.ELEVATOR_MOTOR2.id, Constants.ELEVATOR_MOTOR2.busName);
    elevatorMotor2.setControl(new StrictFollower(elevatorMotor1.getDeviceID()));
    limitSwitch = new DigitalInput(Constants.CLIMBER_SWITCH_LEFT);
    negative = 1;
    
    SetUpClimberMotors();
    
    setVoltage = new PositionDutyCycle(0).withSlot(0);
    setVoltage.UpdateFreqHz = 10;
    setVoltage2 = new PositionDutyCycle(0).withSlot(0);
    setVoltage2.UpdateFreqHz = 10;
  }
  public void SetUpClimberMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    TalonFXConfiguration config2 = new TalonFXConfiguration();

    config.Slot0.kP = 0.045; // p pid
    config.Slot0.kD = 0.00005; // d pid

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.PeakForwardDutyCycle = 1;
    config.MotorOutput.PeakReverseDutyCycle = -1; // can bump up to 12 or something
    config2.Slot0.kP = 0.435; // p pid
    config2.Slot0.kD = 0.00005; // d pid

    config2.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config2.MotorOutput.PeakForwardDutyCycle = 1;
    config2.MotorOutput.PeakReverseDutyCycle = -1; // can bump up to 12 or something

    elevatorMotor1.getConfigurator().apply(config);
    elevatorMotor2.getConfigurator().apply(config2);
  }
  
  public void elevatorOn(double desiredEncoderValue){
    desiredEncoder = desiredEncoderValue;

    double variable = pidController.calculate(getEncoderValue(),desiredEncoderValue);
    if(variable<0){
      negative = -1;
    }
    else{
      negative = 1;
    }

    variable = negative*Math.min(7.2, Math.abs(variable));
    elevatorMotor1.set(variable);  
    
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
  
  public void zeroEncoder1() {
    elevatorMotor1.setPosition(0);
  }

  public void zeroEncoder2() {
    elevatorMotor2.setPosition(0);
  }
  
  public boolean getLimitSwitch(){
    return limitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("ElevatorSwitch", getLimitSwitch());
    SmartDashboard.putNumber("rightEncoder", getEncoderValue());
    SmartDashboard.putNumber("leftEncoder", getEncoderValue2());
    
    zeroEncoder2();
    zeroEncoder1();

    elevatorMotor1.getPosition().refresh();
    elevatorMotor2.getPosition().refresh();
  
  }
}
