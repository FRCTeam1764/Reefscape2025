// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstantsYAGSL;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToTarget extends Command {

  private LimelightSubsystem m_LimeLight;
  private SwerveSubsystem m_Drivetrain;
  private Pose2d m_Game_Piece_Pose;

  private double m_horizontal_angle;
  
  private double m_speed_modifier = 1;

  private PIDController thetaController = new PIDController(SwerveConstantsYAGSL.Auton.angleAutoPID.kP, SwerveConstantsYAGSL.Auton.angleAutoPID.kI, SwerveConstantsYAGSL.Auton.angleAutoPID.kD);

  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(2.8, 1.5);
  private ProfiledPIDController xController = new ProfiledPIDController(4,0.0,0.3, constraints);
  private ProfiledPIDController yController = new ProfiledPIDController(4,0.0,0.3, constraints);


  public DriveToTarget(SwerveSubsystem drivetrain, LimelightSubsystem limelight) {
    addRequirements(drivetrain);
    m_Drivetrain = drivetrain;
    m_LimeLight = limelight;
  }

  public DriveToTarget(SwerveSubsystem drivetrain, LimelightSubsystem limelight, double speed_modifier) {
    addRequirements(drivetrain);
    m_Drivetrain = drivetrain;
    m_LimeLight = limelight;
    m_speed_modifier = speed_modifier;
  }
  

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.reset();
    thetaController.setTolerance(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double thetaOutput = 0;
    double xOutput = 0;
    double yOutput = 0;
    
  	if (m_LimeLight.hasTarget()){
      // prev InterpolationConstants.GAME_PIECE_INTERPOLATOR.getInterpolatedValue(m_LimeLight.getTy().getDouble(0.0)
      //this probably wont work 
      Translation2d t = new Translation2d(m_LimeLight.getTy().getDouble(0), 0);
      Rotation2d r = new Rotation2d(m_Drivetrain.getHeading().getRadians()+m_LimeLight.getTxAngleRadians());
      Transform2d i = new Transform2d(t, r);
      m_Game_Piece_Pose = m_Drivetrain.getPose().transformBy(i);

      m_horizontal_angle = m_LimeLight.getHorizontalAngleOfErrorDegrees();
    } else {
			System.out.println("NO TARGET");
		}

    //xController.setSetpoint(setpoint_x); when it was a normal pid controller
		xOutput = xController.calculate(m_Game_Piece_Pose.getX(), m_Drivetrain.getPose().getX());
    yOutput = yController.calculate(m_Game_Piece_Pose.getY(), m_Drivetrain.getPose().getY());

		double setpoint = m_Drivetrain.getPose().getRotation().getRadians() - Math.toRadians(m_horizontal_angle);
    thetaController.setSetpoint(setpoint);
    if (!thetaController.atSetpoint() ){
			thetaOutput = thetaController.calculate(m_Drivetrain.getPose().getRotation().getRadians(), setpoint);
		}

    SmartDashboard.putNumber("xOutput", xOutput);
    SmartDashboard.putNumber("yOutput", yOutput);
    SmartDashboard.putNumber("Robot Angle", Math.toDegrees(m_Drivetrain.getPose().getRotation().getRadians()));
    SmartDashboard.putNumber("Calculated Angle", Math.toDegrees(setpoint));

    m_Drivetrain.drive(new Translation2d(xOutput * m_Drivetrain.maximumSpeed,yOutput*m_Drivetrain.maximumSpeed),thetaOutput, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_Drivetrain.drive(new Translation2d(0,0), 0, interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
