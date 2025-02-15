// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommand extends Command {
  /** Creates a new IntakeCommand. */
  IntakeRollers intake;
  double speed;
  boolean stopAtLimit;
  public IntakeCommand(IntakeRollers intake, double speed, boolean stopAtLimit) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.speed = speed;
    this.stopAtLimit = stopAtLimit;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.wheelsIntake(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.wheelsIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (stopAtLimit && intake.getIntakeLimitSwitch()) {
      return true;
    } else {
      return false;
    }
  }
}