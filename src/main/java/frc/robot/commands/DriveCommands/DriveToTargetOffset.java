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


public class DriveToTargetOffset extends Command {
  private LimelightSubsystem m_Limelight;
  private CommandSwerveDrivetrain m_Drivetrain;
  private Integer m_pipeline;
  private PIDController xController = new PIDController(0.02, CommandConstants.driveKi, 0 );
  private PIDController yController = new PIDController(0.11, CommandConstants.driveKi, 0);
  private double targetx;
  private double targety;

  private boolean targeting = false;
  private double offset;
  public DriveToTargetOffset(CommandSwerveDrivetrain drivetrain, LimelightSubsystem Limelight, double offset, int pipeline, double targetx, double targety) {
    addRequirements(drivetrain);
    m_Drivetrain = drivetrain;
    m_Limelight = Limelight;
    m_pipeline = pipeline;
    this.targetx = targetx;
    this.targety=targety;
    xController.setSetpoint(targetx);
    yController.setSetpoint(targety);
    this.offset = offset;
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
    xController.reset();
    xController.setTolerance(1);
    yController.reset();
    yController.setTolerance(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("DriveToLLTarget running", true);
    double xSpeed = 0;
    double ySpeed = 0;
		if (m_Limelight.hasTarget()){
			double z_distance = m_Limelight.getTa(); //THIS IS IN TERMS OF CAMERA WATCHOUT
			double x_distance = m_Limelight.getTx();
			 xSpeed = xController.calculate(x_distance, targetx);
       ySpeed = yController.calculate(z_distance, targety);
       SmartDashboard.putNumber("Alimelight", ySpeed);
       SmartDashboard.putNumber("Xlimelight", xSpeed);


      //xOutput = -m_throttle.get()*DrivetrainConstants.maxSpeedMetersPerSecond;
		
		} 
    
    m_Drivetrain.setControl(drive.withVelocityX(ySpeed*(CommandConstants.MaxSpeed/3)).withVelocityY(xSpeed*(CommandConstants.MaxSpeed/3)).withRotationalRate(0));
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
    return false; //TODO: FINISH CHECKINHG
    //return Math.abs(m_Limelight.getTa()-targety) <= 0.4 && Math.abs(m_Limelight.getTx()-targetx)<=0.25;
  }
}