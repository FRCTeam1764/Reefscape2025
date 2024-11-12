// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.SwerveConstantsYAGSL;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class LockOnAprilTag extends Command {
  /** Creates a new LockOnAprilTag. */
   
  private LimelightSubsystem LimeLight;
  private SwerveSubsystem Drivetrain;
private boolean fieldRelative;
  private Joystick controller;
  private boolean gottadoFunny;
  private int pipeline;
  private PIDController thetaController = new PIDController(SwerveConstantsYAGSL.Auton.angleAutoPID.kP, SwerveConstantsYAGSL.Auton.angleAutoPID.kI, SwerveConstantsYAGSL.Auton.angleAutoPID.kD);
  public LockOnAprilTag(SwerveSubsystem drivetrain, LimelightSubsystem limelight, int pipeline, Joystick controller,boolean fieldRelative, boolean gottadoFunny) {
    addRequirements(drivetrain);
    this.Drivetrain = drivetrain;
    this.LimeLight = limelight;
    this.controller = controller;
    this.fieldRelative = fieldRelative;
    this.pipeline = pipeline;
    //gottadofunny represnets the time in GKC where limeight couldnt see the april tag so we hvae to use another
   this.gottadoFunny = gottadoFunny;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.reset();
    thetaController.setTolerance(Math.toRadians(1)); //fix later?
    LimeLight.setPipeline(pipeline);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    double thetaOutput = 0;
    double xOutput =MathUtil.applyDeadband(-controller.getRawAxis(XboxController.Axis.kLeftY.value),SwerveConstantsYAGSL.OperatorConstants.LEFT_X_DEADBAND)*Drivetrain.maximumSpeed;
    double yOutput = MathUtil.applyDeadband(-controller.getRawAxis(XboxController.Axis.kLeftX.value),SwerveConstantsYAGSL.OperatorConstants.RIGHT_X_DEADBAND)*Drivetrain.maximumSpeed;
    double horizontal_amgle = -(LimeLight.getHorizontalAngleOfErrorDegrees());

		if (LimeLight.hasTarget()){
      // if(gottadoFunny == true){
      //   if(LimeLight.getID() == 3 || LimeLight.getID() == 8){
      //     LimeLight.setPipeline(2);
      //      horizontal_amgle = -(LimeLight.getHorizontalAngleOfErrorDegrees()+22 );
      //   }else{

      //     LimeLight.setPipeline(1);
      //   }
      // }
      



			double setpoint = Math.toRadians(horizontal_amgle)+Drivetrain.getPose().getRotation().getRadians();
      thetaController.setSetpoint(setpoint);

			if (!thetaController.atSetpoint()){
				thetaOutput = thetaController.calculate(Drivetrain.getPose().getRotation().getRadians(), setpoint);
			}
      System.out.print(String.valueOf(thetaOutput));
		} 
    Drivetrain.drive(new Translation2d(xOutput,yOutput),thetaOutput,fieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LimeLight.setPipeline(pipeline);
    Drivetrain.drive(new Translation2d(0,0),0,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}