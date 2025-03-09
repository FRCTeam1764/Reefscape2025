// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.IntakeWristRev;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristCommand extends Command {
  /** Creates a new WristCommand. */
  IntakeWristRev intake;
  int desired;
  public WristCommand(IntakeWristRev intake, int desired) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.desired = desired;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.flexOn(desired);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopflex();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.intake.getEncoderPos() <= this.desired+2 && this.intake.getEncoderPos() >= this.desired-2;
  }
}
