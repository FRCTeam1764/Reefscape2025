// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CommandConstants;
import frc.robot.constants.Constants;

//TODO: FIND CONSTANTS FOR Limits, PID
public class IntakeWrist extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX m_flexMotor = new TalonFX(Constants.WRIST_MOTOR1.id, Constants.WRIST_MOTOR1.busName);
  final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);


  ArmFeedforward armfeed = new ArmFeedforward(0,0.3,0);

  private final CANcoder m_angleEncoder = new CANcoder(Constants.INTAKE_CANCODER.id, Constants.INTAKE_CANCODER.busName);
  private StateManager stateManager;

  public IntakeWrist( StateManager stateManager) {

    //configs
    this.stateManager = stateManager;


    //TODO figure out values
    TalonFXConfiguration flexConfig = new TalonFXConfiguration(); //TODO chack all of it
    flexConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
    flexConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flexConfig.MotorOutput.PeakForwardDutyCycle = 0.9;
    flexConfig.MotorOutput.PeakReverseDutyCycle = -0.9; 
    flexConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flexConfig.CurrentLimits.StatorCurrentLimit = 60;

    flexConfig.Feedback.FeedbackRemoteSensorID = Constants.INTAKE_CANCODER.id;
    flexConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;


    flexConfig.Slot0.kP = 0;
    flexConfig.Slot0.kD = 0;


    CANcoderConfiguration CANcoderConfigs = new CANcoderConfiguration();
    CANcoderConfigs.MagnetSensor.withMagnetOffset(0); //offset
    CANcoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1; //NOTE: its range is currently [0, 1) 

    m_angleEncoder.getConfigurator().apply(CANcoderConfigs);
    m_flexMotor.getConfigurator().apply(flexConfig);

  }

  public void flexOn(double rotations) {
    m_flexMotor.setControl((m_request.withPosition(rotations)));
  }

  public void startflex1(){
    m_flexMotor.set(.2);
  }

  public void stopflex(){
    m_flexMotor.set(0);
  }


  public double getEncoderPos() {
    return m_angleEncoder.getPosition().getValueAsDouble();
  }
  
  //TODO: find not safe encoder values
  // public boolean isFlexSafe(){
  //   return Math.abs(getEncoderPos()) > 180 && Math.abs(getEncoderPos()) <90;
  // }

 


  public boolean isFlexSafe(){
 
    double elevatorCurrentPos = (double) stateManager.getCurrentData("ElevatorPosition");

    double wristCurrentPos = (double) stateManager.getCurrentData("WristEncoderPosition");
    

return ( elevatorCurrentPos < 3 ) || (elevatorCurrentPos < 10 && wristCurrentPos < 30);
  }



  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakeWristPosition", m_angleEncoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("IntakeWristTempature",m_flexMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("IntakeWristCurrent", m_flexMotor.getStatorCurrent().getValueAsDouble());


    
    stateManager.updateCurrentData("WristEncoderPosition",  m_angleEncoder.getPosition().getValueAsDouble());

    
  }
}