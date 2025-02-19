// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;


import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CommandConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;


public class DriveToTarget extends Command {
  private LimelightSubsystem m_Limelight;
  private CommandSwerveDrivetrain m_Drivetrain;
  private Integer m_pipeline;
  private PIDController thetaController = new PIDController(CommandConstants.kP, CommandConstants.kI, CommandConstants.kD);
  private boolean targeting = false;
  public DriveToTarget(CommandSwerveDrivetrain drivetrain, LimelightSubsystem Limelight, int pipeline) {
    addRequirements(drivetrain);
    m_Drivetrain = drivetrain;
    m_Limelight = Limelight;
    m_pipeline = pipeline;
  }

  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_Limelight.setPipeline(m_pipeline);
    targeting = false;
    thetaController.reset();
    thetaController.setTolerance(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("DriveToLLTarget running", true);
    double thetaOutput = 0;
    double xOutput = 0;
		if (m_Limelight.hasTarget()){
			double vertical_angle = m_Limelight.getVerticalAngleOfErrorDegrees();
			double horizontal_angle = -m_Limelight.getHorizontalAngleOfErrorDegrees() ;
			double setpoint = Math.toRadians(horizontal_angle)+ m_Drivetrain.getPose().getRotation().getRadians();
      thetaController.setSetpoint(setpoint);
      targeting = true;
		
			thetaOutput = thetaController.calculate(m_Drivetrain.getPose().getRotation().getRadians(), setpoint);
      //xOutput = -m_throttle.get()*DrivetrainConstants.maxSpeedMetersPerSecond;
		
      SmartDashboard.putNumber("targeting error", horizontal_angle);
		} 
    
    m_Drivetrain.setControl(drive.withVelocityX(0).withRotationalRate(thetaOutput));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("DriveToLLTarget running", true);
    m_Drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}