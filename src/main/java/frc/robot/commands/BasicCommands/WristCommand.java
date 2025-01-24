// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristCommand extends Command {
  /** Creates a new WristCommand. */
  IntakeSubsystem intake;
  int desired;
  public WristCommand(IntakeSubsystem intake, int desired) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.desired = desired;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.startflex1();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.flexClosedLoop(desired);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopflex1();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.intake.getEncoderPos() <= this.desired && this.intake.getEncoderPos() >= this.desired && intake.isFlexSafe();
  }
}
