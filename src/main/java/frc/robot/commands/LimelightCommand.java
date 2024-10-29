// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.SwerveConstants;
import frc.robot.state.LimelightState;
import frc.robot.state.SwerveState;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve;

public class LimelightCommand extends CommandBase {
  LimelightState limelightState;
  private LimelightSubsystem limelight;
  private int pipeline;
  public boolean limelightOn;
  private Swerve swerve;
  private SwerveState  swerveState;
  
  
  /** Creates a new LimelightCommand. */
  public LimelightCommand(LimelightSubsystem limelight, int pipeline,Swerve swerve,SwerveState swerveState,LimelightState limelightState) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.pipeline = pipeline;
this.swerve = swerve;
this.swerveState = swerveState;
this.limelightState = limelightState;
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("hi world");
    limelightState.limelightOn();
    swerveState.swerveAutoAlign();

    limelight.setPipeline(pipeline);


  }

public double getoffset(){
System.out.println("it ot here");
  if (limelight.updateIsThereTarget() == 0){
      swerveState.noSwerveAutoAlign();
      return 0;
  }
if (limelight.whereToMove() <= 1.5){
  swerveState.noSwerveAutoAlign();
  return 0;
}

  return limelight.whereToMove();
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //  limelight.setPipeline(pipeline);


    if (swerveState.swerveAutoAlign == true){
    swerve.drive(new Translation2d( -getoffset(),0).times(SwerveConstants.Swerve.maxSpeed),
        0, 
        false,
        true
      );
          }

    // limelight.updateXOffset();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
limelightState.limelightOff(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return swerveState.swerveAutoAlign;
  }
}
