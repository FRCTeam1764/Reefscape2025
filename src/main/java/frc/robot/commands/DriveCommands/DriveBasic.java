// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.libraries.lib.util.COTSFalconSwerveConstants.driveGearRatios;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveBasic extends Command {
  /** Creates a new DriveBasic. */
  double speed;
  SwerveSubsystem swerve;
  LimelightSubsystem limeLight;
  boolean goToLimelightpos = false;
  double pos;
  public DriveBasic(SwerveSubsystem swerve, LimelightSubsystem limeLight, boolean goToLimelightpos, double pos, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.limeLight = limeLight;
    this.goToLimelightpos = goToLimelightpos;
    this.speed = speed;
    this.pos = pos;
    addRequirements(swerve,limeLight);
  }
    public DriveBasic(SwerveSubsystem swerve, double speed) {
      this.swerve = swerve;
      this.speed = speed;
      addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
swerve.drive(new Translation2d(-speed *swerve.maximumSpeed,0), 0 ,false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0,0), 0 ,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(goToLimelightpos){
    return  Math.abs(limeLight.getTy().getDouble(0)) < pos +1 && Math.abs(limeLight.getTy().getDouble(0)) < pos - 1;
    }
    return false;
  }
}
