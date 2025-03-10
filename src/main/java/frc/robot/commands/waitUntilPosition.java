// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.StateManager;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class waitUntilPosition extends Command {
  /** Creates a new waitUntilCondition. */
    //TODO: MAKE

StateManager stateManager;
String key;
double error;
String key2;
double error2;

  public waitUntilPosition(StateManager manager, String key1, double error1, String key2, double error2) {
    // Use addRequirements() here to declare subsystem dependencies.
    stateManager = manager;
    this.key = key1;
    this.error = error1;
    this.key2 = key2;
    this.error2 = error2;
  }

  public waitUntilPosition(StateManager manager) {
    // Use addRequirements() here to declare subsystem dependencies.
    stateManager = manager;
    key = CommandConstants.ELEVATOR_KEY;
    error = 4;
    key2 = CommandConstants.INTAKE_KEY;
    error2 = 4;

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
    return ( (double) stateManager.getCurrentData(key) <=  (double) stateManager.getDesiredData(key) +error
     ||
    (double) stateManager.getCurrentData(key) <= (double) stateManager.getDesiredData(key) -error
     ) && ( (double) stateManager.getCurrentData(key2) <=  (double) stateManager.getDesiredData(key2) +error2
     ||
    (double) stateManager.getCurrentData(key2) <= (double) stateManager.getDesiredData(key2) -error2
     );
  }
}
