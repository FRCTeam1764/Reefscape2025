// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CommandConstants;
import frc.robot.constants.Constants;

public class IntakeWristRev extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private  TalonFX m_flexMotor = new TalonFX(Constants.WRIST_MOTOR1.id, Constants.WRIST_MOTOR1.busName);
   PositionVoltage m_request = new PositionVoltage(0).withSlot(0);


  ArmFeedforward armfeed = new ArmFeedforward(0,0.3,0);

  private SparkMax jank = new SparkMax(0, null); //this is bad, dont do this future coders!
  private AbsoluteEncoder revAbsoluteEncoder = jank.getAbsoluteEncoder(); //to run a rev encoder through a talon, we need to get it through the spark max and do external pid calculations. this will stay uintil we get a actual ctre-throughboreencoder

  private PIDController controller = new PIDController(0.001, 0, 0.0001);


  public IntakeWristRev() {

  //configs


    //TODO figure out values
    TalonFXConfiguration flexConfig = new TalonFXConfiguration(); //TODO chack all of it
    flexConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
    flexConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flexConfig.MotorOutput.PeakForwardDutyCycle = 0.7;
    flexConfig.MotorOutput.PeakReverseDutyCycle = -0.7; 
    flexConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flexConfig.CurrentLimits.StatorCurrentLimit = 60;



    SparkMaxConfig jankconfig = new SparkMaxConfig();


    AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
    encoderConfig.positionConversionFactor(360);
    encoderConfig.zeroOffset(0);

    

    jankconfig.apply(encoderConfig);
    jank.configure(jankconfig, null, null);

    m_flexMotor.getConfigurator().apply(flexConfig);

  }

  public void flexOn(double rotations) {
    double calculation = controller.calculate(revAbsoluteEncoder.getPosition(), rotations);
    SmartDashboard.putNumber("INTAKE_WRIST_PID", calculation);
    m_flexMotor.set(calculation);
  }

  public void startflex1(){
    m_flexMotor.set(.2);
  }

  public void stopflex(){
    m_flexMotor.set(0);
  }

  public double getEncoderPos() {
    return revAbsoluteEncoder.getPosition();
  }
  

  
  //TODO: find not safe encoder values
  // public boolean isFlexSafe(){
  //   return Math.abs(getEncoderPos()) > 180 && Math.abs(getEncoderPos()) <90;
  // }




  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakeWristPosition",revAbsoluteEncoder.getPosition());
    SmartDashboard.putNumber("IntakeWristTempature",m_flexMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("IntakeWristCurrent", m_flexMotor.getStatorCurrent().getValueAsDouble());


    
    
  }
}


