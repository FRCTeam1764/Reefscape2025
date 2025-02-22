// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

public class LockOnAprilTag extends Command {
  private LimelightSubsystem m_Limelight;
  private frc.robot.subsystems.CommandSwerveDrivetrain m_Drivetrain;
  private int m_pipeline;
  private PIDController thetaController = new PIDController(.4, 0, .1);
  private boolean targeting = false;
  private CommandXboxController controller;
  private Supplier<Double> m_skew ; 
  //(swerve, Limelight2, 0, driver, false
  public LockOnAprilTag(CommandSwerveDrivetrain drivetrain, LimelightSubsystem Limelight, int pipeline, CommandXboxController controller, boolean robotcentric) {
    addRequirements(drivetrain);
    m_Drivetrain = drivetrain;
    m_Limelight = Limelight;
    m_pipeline = pipeline;
    this.controller = controller;
   // m_skew = skewDegrees;
  }
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDeadband(MaxSpeed * 0.1);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Limelight.setPipeline(m_pipeline);
    targeting = false;
    thetaController.reset();
    thetaController.setTolerance(Math.toRadians(1.5));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("AllignOnLLTarget running", true);
    double thetaOutput = 0;
    double xOutput = controller.getLeftX();
    double yOutput = controller.getLeftY();
		if (m_Limelight.hasTarget()){
			double vertical_angle = m_Limelight.getHorizontalAngleOfErrorDegrees();
			double horizontal_angle = -m_Limelight.getVerticalAngleOfErrorDegrees() ;
			double setpoint = Math.toRadians(horizontal_angle)+ m_Drivetrain.getPose().getRotation().getRadians() + Math.toRadians(m_skew.get());
      thetaController.setSetpoint(setpoint);
      targeting = true;
			if (!thetaController.atSetpoint() ){
				thetaOutput = thetaController.calculate(m_Drivetrain.getPose().getRotation().getRadians(), setpoint);
			} 
      SmartDashboard.putNumber("targeting error", horizontal_angle);
		} 
    else {
			System.out.println("NO TARGET");
		}
    m_Drivetrain.setControl(drive.withVelocityX(xOutput).withVelocityY(yOutput).withRotationalRate(thetaOutput));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("AllignOnLLTarget running", false);
    m_Drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return targeting &&  Math.abs(m_Limelight.getVerticalAngleOfErrorDegrees() ) <= 3;
  }
}