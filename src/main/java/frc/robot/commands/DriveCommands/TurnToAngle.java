
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.constants.CommandConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TurnToAngle extends Command {
  /** Creates a new TurnToAngle. */
 	private CommandSwerveDrivetrain m_drivetrain;
	private CommandXboxController m_gamepad;
	private SlewRateLimiter xLimiter = new SlewRateLimiter(2.5);
	private SlewRateLimiter yLimiter = new SlewRateLimiter(2.5);
	private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3);
	private PIDController thetaController = new PIDController(6, CommandConstants.kI, CommandConstants.kD);
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private double m_angle;
 

	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed   * .05).withRotationalDeadband(MaxAngularRate* .05) // Add a 5% deadband
      .withRotationalDeadband(0); // I want field-centric
                                                               // driving in open loop
															   // voltage mode	
 	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  	//private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	/**
	 * Constructor method for the GamepadDrive class
	 * - Creates a new GamepadDrive object.
	 */
	public TurnToAngle(CommandSwerveDrivetrain drivetrain, Double angle) {
		super();
		addRequirements(drivetrain);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
		m_drivetrain = drivetrain;
		m_angle = angle;
	}

	@Override
	public void execute() {
		

		
    double thetaOutput = 0;
		double horizontal_angle = m_angle;
		double setpoint = Math.toRadians(horizontal_angle);
      	
		thetaController.setSetpoint(setpoint);
		//if (!thetaController.atSetpoint() ){
			thetaOutput = thetaController.calculate(m_drivetrain.getPose().getRotation().getRadians(), setpoint);
		//} 

    m_drivetrain.setControl(drive
			.withVelocityX(0)
			.withVelocityY(0) 
			.withRotationalRate(thetaOutput));
	
	 }

	@Override
	public void end(boolean interrupted) {
		m_drivetrain.applyRequest(() -> brake);
	}


}
