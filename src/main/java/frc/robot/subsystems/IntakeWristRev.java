// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;

//TODO: FIND CONSTANTS FOR Limits, PID
public class IntakeWristRev extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private  TalonFX m_flexMotor = new TalonFX(Constants.WRIST_MOTOR1.id, Constants.WRIST_MOTOR1.busName);
   PositionVoltage m_request = new PositionVoltage(0).withSlot(0);


  ArmFeedforward armfeed = new ArmFeedforward(0,0.3,0);

  private SparkMax jank = new SparkMax(5,MotorType.kBrushless); //this is bad, dont do this future coders!
  private AbsoluteEncoder revAbsoluteEncoder = jank.getAbsoluteEncoder(); //to run a rev encoder through a talon, we need to get it through the spark max and do external pid calculations. this will stay uintil we get a actual ctre-throughboreencoder

  private PIDController controller = new PIDController(0.0075, 0, 0.00001);

  private StateManager stateManager;

  private VoltageOut voltageOut;
  private double calculation;

  private final SysIdRoutine m_sysIdRoutine =
   new SysIdRoutine(
      new SysIdRoutine.Config(
         null,        // Use default ramp rate (1 V/s)
         Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
         null,        // Use default timeout (10 s)
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> m_flexMotor.setControl(voltageOut.withOutput(volts.in(Volts))),
         null,
         this
      )
   );

  public IntakeWristRev(StateManager stateManager) {

    this.stateManager = stateManager;
    
    //configs

    //TODO figure out values
    TalonFXConfiguration flexConfig = new TalonFXConfiguration(); //TODO chack all of it
    flexConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
    flexConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    flexConfig.MotorOutput.PeakForwardDutyCycle = 0.5;
    flexConfig.MotorOutput.PeakReverseDutyCycle = -0.5; 
    flexConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flexConfig.CurrentLimits.StatorCurrentLimit = 60;



    SparkMaxConfig jankconfig = new SparkMaxConfig();
    SmartDashboard.putNumber("rotationswrist", 40);

    AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig(); 
    encoderConfig.inverted(true);
    encoderConfig.zeroOffset(110.0/360);
    encoderConfig.positionConversionFactor(360);
    


    jankconfig.absoluteEncoder.apply(encoderConfig);
    jank.configure(jankconfig, null, null);

    m_flexMotor.getConfigurator().apply(flexConfig);

  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public void flexOn(double rotations) {
    if (getEncoderPos() > 300 || getEncoderPos() < 10 ){
      m_flexMotor.set(0);
    } else {
      calculation = controller.calculate(revAbsoluteEncoder.getPosition(), rotations);
      SmartDashboard.putNumber("INTAKE_WRIST_PID", calculation);
      m_flexMotor.set(calculation);
    }
  }

  public void startflex1(){
    m_flexMotor.set(.1);
  }

  public void stopflex(){
    m_flexMotor.set(0);
  }

  public double getEncoderPos() {
    return revAbsoluteEncoder.getPosition();
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakeWristPosition",jank.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("IntakeWristTempature",m_flexMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("IntakeWristCurrent", m_flexMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putBoolean("WristHappy", m_flexMotor.getStatorCurrent().getValueAsDouble()<30);
    SmartDashboard.putNumber("UnhappyCount", SmartDashboard.getNumber("UnhappyCount", 0) + (SmartDashboard.getBoolean("ElevatorHappy", true) ? 0: 1));
    
    stateManager.updateCurrentData("WristEncoderPosition",revAbsoluteEncoder.getPosition());
  }
}