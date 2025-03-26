// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StateManager;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class waitUntilPositionIndex extends Command {
  /** Creates a new waitUntilCondition. */

  StateManager stateManager;
  String key;
  double error;
  double index;

  public waitUntilPositionIndex(StateManager manager, String key, double error, double index) {
    // Use addRequirements() here to declare subsystem dependencies.
    stateManager = manager;
    this.key = key;
    this.error = error;
    this.index = index;
  }

  public waitUntilPositionIndex(StateManager manager, String key, double index) {
    // Use addRequirements() here to declare subsystem dependencies.
    stateManager = manager;
    this.key = key;
    error = .5;
    this.index = index;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ( (double) stateManager.getCurrentData(key) <= index +error &&
             (double) stateManager.getCurrentData(key) >= index -error
    );
  }
}
