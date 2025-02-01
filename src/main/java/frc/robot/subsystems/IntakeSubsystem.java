// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CommandConstants;
import frc.robot.constants.Constants;

//TODO: FIND CONSTANTS FOR Limits, PID
public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX m_flexMotor = new TalonFX(Constants.WRIST_MOTOR1.id, Constants.WRIST_MOTOR1.busName);
  private final TalonFX m_intakeMotor = new TalonFX(Constants.INTAKE_MOTOR.id);
  private SparkClosedLoopController pidController;

  ArmFeedforward armfeed = new ArmFeedforward(0,0.3,0);

  private final CANcoder m_angleEncoder = new CANcoder(Constants.INTAKE_CANCODER.id, Constants.INTAKE_CANCODER.busName);
  private DigitalInput breakBeamIntake;


  public IntakeSubsystem() {

  //configs

    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.Inverted =  InvertedValue.Clockwise_Positive;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeConfig.MotorOutput.PeakForwardDutyCycle = 0.8;
    intakeConfig.MotorOutput.PeakReverseDutyCycle = 0.8;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.StatorCurrentLimit = 60;

    TalonFXConfiguration flexConfig = new TalonFXConfiguration();
    flexConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    flexConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    CANcoderConfiguration CANcoderConfigs = new CANcoderConfiguration();
    CANcoderConfigs.MagnetSensor.withMagnetOffset(0); //offset
    CANcoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1; //NOTE: its range is currently [0, 1) 

    m_angleEncoder.getConfigurator().apply(CANcoderConfigs);
    m_intakeMotor.getConfigurator().apply(intakeConfig);  
    m_flexMotor.getConfigurator().apply(flexConfig);

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

  public boolean getIntakeBreakbeam() {
    return !breakBeamIntake.get();
  }

  public void run(double speed) {
    
   m_intakeMotor.set(speed);
   
  }

  public double getPercentFromBattery(double speed){
    return speed * 12 / RobotController.getBatteryVoltage();
  }

  public void stop() {
    m_intakeMotor.set(0);
  }

  public void startflex1(){
    m_flexMotor.set(.2);
  }

  public void stopflex1(){
    m_flexMotor.set(0);
  }
  public void flexClosedLoop(double desired) {
      pidController.setReference(desired, ControlType.kPosition);
  }

  public double getEncoderPos() {
    return m_angleEncoder.getPosition().getValueAsDouble();
  }
  
  //TODO: find not safe encoder values
  public boolean isFlexSafe(){
    return Math.abs(getEncoderPos()) > 180 && Math.abs(getEncoderPos()) <90;
  }

  public boolean flexOutSafe(ElevatorSubsystem elevator, IntakeSubsystem intake) {
    return elevator.getEncoderValue() >= CommandConstants.ELEVATOR_STOP_SAFE || 
      ((int)intake.getEncoderPos() >= CommandConstants.WRIST_DOWN-1 && (int)intake.getEncoderPos() <= CommandConstants.WRIST_DOWN+1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("IntakeBreakbeam", getIntakeBreakbeam());
    SmartDashboard.putNumber("IntakeCurrent", m_intakeMotor.getMotorStallCurrent().getValueAsDouble());
    SmartDashboard.putNumber("IntakeWristPosition", m_angleEncoder.getPosition().getValueAsDouble());
    
  }
}
