// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveBackward extends Command {
  /** Creates a new DriveForward. */
  CommandSwerveDrivetrain drivetrain;
  SwerveRequest.RobotCentric driveForward = new SwerveRequest.RobotCentric();
  
  public DriveBackward(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 drivetrain.setControl(driveForward.withVelocityX(CommandConstants.MaxSpeed * -.1 ).withVelocityY(0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(driveForward.withVelocityX(0).withVelocityY(0));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
