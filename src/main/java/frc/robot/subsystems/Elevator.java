// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.CommandConstants;
import frc.robot.constants.Constants;
import com.ctre.phoenix6.hardware.TalonFX;


public class Elevator extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  public TalonFX elevatorMotor1;
  public TalonFX elevatorMotor2;
  public DigitalInput limitSwitchTop1;
  public DigitalInput limitSwitchTop2;

  public DigitalInput limitSwitchBottom1;
  public DigitalInput limitSwitchBottom2;
  private VoltageOut voltageOut = new VoltageOut(0.0);
  private double maxdutycycle = 0.25;
  


  private MotionMagicVoltage setVoltage;
  private double desiredEncoder;
  private StateManager stateManager;
  int negative;
// try using these
  public static final double MOTION_MAGIC_ACCELERATION = 120;//80
  public static final double MOTION_MAGIC_VELOCITY = 170;//100

  private final SysIdRoutine m_sysIdRoutine =
   new SysIdRoutine(
      new SysIdRoutine.Config(
         null,        // Use default ramp rate (1 V/s)
         Volts.of(1), // Reduce dynamic step voltage to 4 to prevent brownout
         null,        // Use default timeout (10 s)
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> elevatorMotor1.setControl(voltageOut.withOutput(volts.in(Volts))),
         null,
         this
      )
   );

  public Elevator(StateManager stateManager) {
    elevatorMotor1 = new TalonFX(Constants.ELEVATOR_MOTOR1.id, Constants.ELEVATOR_MOTOR1.busName);
    elevatorMotor2 = new TalonFX(Constants.ELEVATOR_MOTOR2.id, Constants.ELEVATOR_MOTOR2.busName);
    limitSwitchBottom1 = new DigitalInput(Constants.ELEVATOR_SWITCHDOWN1);
    limitSwitchBottom2 = new DigitalInput(Constants.ELEVATOR_SWITCHDOWN2);

    limitSwitchTop1 = new DigitalInput(Constants.ELEVATOR_SWITCHUP1);
    limitSwitchTop2 = new DigitalInput(Constants.ELEVATOR_SWITCHUP2);
    SmartDashboard.putNumber("elevatorencoder", 0);

    
    
    SetUpClimberMotors();
    
    setVoltage = new MotionMagicVoltage(0).withSlot(0);
    setVoltage.UpdateFreqHz = 10;

    this.stateManager = stateManager;
  }

  public void SetUpClimberMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    TalonFXConfiguration config2 = new TalonFXConfiguration();

    config.Slot0.kP = 3.75; // p pid //4.1
    config.Slot0.kD = 0;//SmartDashboard.getNumber("d", 0.51); // d pid .5362, then .52
    config.Slot0.kV = 0;
    config.Slot0.kA = 0;
    config.Slot0.kG = 0.65;
    

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.PeakForwardDutyCycle = maxdutycycle;
    config.MotorOutput.PeakReverseDutyCycle = -.05; // can bump up to 12 or something
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 60; 
 
  
    config2.Slot0.kP = 3.75; // p pid //4.1
    config2.Slot0.kD = 0;//SmartDashboard.getNumber("d", 0.51); // d pid .5362, then .52
    config2.Slot0.kV = 0;
    config2.Slot0.kA = 0;
    config2.Slot0.kG = 0.65;


    config2.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config2.MotorOutput.PeakForwardDutyCycle = maxdutycycle;
    config2.MotorOutput.PeakReverseDutyCycle = -.05; // can bump up to 12 or something
    config2.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //TODO: FIND IF TRUE OR NOT BEFORE U FRY ROBOT
    config2.CurrentLimits.StatorCurrentLimitEnable = true;
    config2.CurrentLimits.StatorCurrentLimit = 60; 


    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.withMotionMagicAcceleration(MOTION_MAGIC_ACCELERATION).withMotionMagicCruiseVelocity(MOTION_MAGIC_VELOCITY);

    elevatorMotor1.getConfigurator().apply(config);
    elevatorMotor2.getConfigurator().apply(config2);
    elevatorMotor1.getConfigurator().apply(motionMagicConfigs);
    elevatorMotor2.getConfigurator().apply(motionMagicConfigs);

    elevatorMotor2.setControl(new StrictFollower(elevatorMotor1.getDeviceID()));

  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
  
  public void elevatorOn(double desiredEncoderValue){
    //TOOD: NEED FEEDFORWARD

    @SuppressWarnings("unused")
    StatusCode val =   elevatorMotor1.setControl(setVoltage.withPosition(desiredEncoderValue).withSlot(0));//.withLimitReverseMotion(true));
  }

  public void elevatorOnSpeed(double speed){
    elevatorMotor1.set(speed);
  }

  // public void motionmagic(){
  //   elevatorMotor1.setControl()
  // }

  public void off() {
    elevatorMotor1.set(0);
  }
  public double getDesiredEncoder(){
    return desiredEncoder;
  }
  public double getEncoderValue() {
    return elevatorMotor1.getPosition().getValueAsDouble(); //all the way 24.8
  }

  public double getEncoderValue2() {
    return elevatorMotor2.getPosition().getValueAsDouble();
  }
  //TODO: find not safe encoder values
  //CHECKS IF ELEVATOR IS TOO LOW OR TOO HIGH, AS WELL AS ENSURING INTAKE WILL NOT BE RAMMED INTO THE GROUND 
  public boolean Motor1IsSafe(){
    return (Math.abs(getEncoderValue()) > 3 && Math.abs(getEncoderValue()) <80) || (getEncoderValue() < 10 && (double) stateManager.getCurrentData("IntakeAngle") < 30 );
  }
  public boolean Motor2IsSafe(){
    return (Math.abs(getEncoderValue2()) > 3 && Math.abs(getEncoderValue2()) <80)|| (getEncoderValue() < 10 && (double) stateManager.getCurrentData("IntakeAngle") < 30 );
  }
  

public boolean getLimitSwitches(){
    if (limitSwitchBottom1.get() && limitSwitchBottom2.get()){
  return true;
    }else  if(!limitSwitchTop1.get() && !limitSwitchTop2.get()){
  return true;
      }
      return false;
}



  public void setEncoders(double EncoderValue){
    elevatorMotor1.setPosition(EncoderValue);
    elevatorMotor2.setPosition(EncoderValue);
  }

  @Override
  public void periodic() {

    //elevatorOn(10);

    SmartDashboard.putNumber("ElevatorMotor1Position", elevatorMotor1.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("ElevatorMotor2Position", elevatorMotor2.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("ElevatorMotor1Temperature", elevatorMotor1.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("ElevatorMotor2Temperature", elevatorMotor2.getDeviceTemp().getValueAsDouble());

    SmartDashboard.putNumber("ElevatorMotor1Current", elevatorMotor1.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("ElevatorMotor2Current", elevatorMotor2.getStatorCurrent().getValueAsDouble());

    SmartDashboard.putNumber("ElevatorMotor1Voltage", elevatorMotor1.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("ElevatorMotor2Voltage", elevatorMotor2.getMotorVoltage().getValueAsDouble());

    SmartDashboard.putBoolean("ElevatorBottomLimit1", limitSwitchBottom1.get());
    SmartDashboard.putBoolean("ElevatorBottomLimit2", limitSwitchBottom2.get());
    SmartDashboard.putBoolean("ElevatorTopLimit1", limitSwitchTop1.get());
    SmartDashboard.putBoolean("ElevatorTopLimit2", limitSwitchTop2.get());

//zeroing encoders
    if (limitSwitchBottom1.get() || limitSwitchBottom2.get()){
      setEncoders(0);
      
    }else if(!limitSwitchTop1.get() || !limitSwitchTop2.get()){
      setEncoders(24.8); //TODO: FIND MAX ENCODER HEIGHT
    }

  
    stateManager.updateCurrentData("ElevatorPosition", elevatorMotor1.getPosition().getValueAsDouble());
 
  }

}
