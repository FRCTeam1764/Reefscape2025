// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

//TODO: FIND CONSTANTS FOR Limits, PID
public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final SparkMax m_flexMotor = new SparkMax(Constants.WRIST_MOTOR1.id, MotorType.kBrushless);
  private final TalonFX m_intakeMotor = new TalonFX(Constants.INTAKE_MOTOR.id);
  private SparkClosedLoopController pidController;

  ArmFeedforward armfeed = new ArmFeedforward(0,0.3,0);

  private final SparkAbsoluteEncoder m_angleEncoder = m_flexMotor
      .getAbsoluteEncoder();
  private DigitalInput breakBeamIntake;


  public IntakeSubsystem() {

  //configs

    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.Inverted =  InvertedValue.Clockwise_Positive;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeConfig.MotorOutput.PeakForwardDutyCycle = 0.8;
    intakeConfig.MotorOutput.PeakReverseDutyCycle = 0.8;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.StatorCurrentLimit = 120;

    TalonFXConfiguration flexConfig = new TalonFXConfiguration();
    flexConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    flexConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;




  m_intakeMotor.getConfigurator().apply(intakeConfig);  
    

    AbsoluteEncoderConfig angleEncoderConfig = new AbsoluteEncoderConfig();
    angleEncoderConfig
                    .zeroOffset(0)
                    .setSparkMaxDataPortConfig()
                    .inverted(false)
                    .positionConversionFactor(360);

    SparkMaxConfig flexConfig = new SparkMaxConfig();
    flexConfig
            .inverted(true) //TODOFINE OUT
            .secondaryCurrentLimit(100)
            .idleMode(IdleMode.kCoast);
    flexConfig.apply(angleEncoderConfig);
    flexConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(0.0, 0.0, 0.0)
            .outputRange(-.9, .9)
            .positionWrappingEnabled(false);

    m_flexMotor.configure(flexConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);    
            
    
    



    breakBeamIntake = new DigitalInput(Constants.INTAKE_BREAK_BEAM);
    // breakBeamIntakeOut = new DigitalInput(Constants.INTAKE_BREAK_BEAM_FEED);
    // breakBeamIntakeMid = new DigitalInput(Constants.INTAKE_BREAK_BEAM_MIDDLE);

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
    return m_angleEncoder.getPosition();
  }
  //TODO: find not safe encoder values
  public boolean isFlexSafe(){
    return Math.abs(getEncoderPos()) > 180 && Math.abs(getEncoderPos()) <90;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("IntakeBreakbeam", getIntakeBreakbeam());
    // SmartDashboard.putBoolean("Intake2", !breakBeamIntakeOut.get());
    // SmartDashboard.putBoolean("Intake3",!breakBeamIntakeMid.get());
    SmartDashboard.putNumber("intakeCurre t", m_intakeMotor.getMotorStallCurrent().getValueAsDouble());
    SmartDashboard.putNumber("intakepos", m_angleEncoder.getPosition());
    
  }
}
