// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DefaultCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.StateManager;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultWristCommand extends Command {
  /** Creates a new DefaultWristCommand. */
  IntakeWrist wrist;
  StateManager stateManager;
  public DefaultWristCommand(IntakeWrist wrist, StateManager stateManager) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist = wrist;
    this.stateManager = stateManager;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
