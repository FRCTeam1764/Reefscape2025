// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommand extends Command {
  /** Creates a new ElevatorCommand. */
  Elevator elevator;
  int desired;
  boolean stopAtLimitSwitch;
  public ElevatorCommand(Elevator elevator, int desired, boolean stopAtLimitSwitch) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.stopAtLimitSwitch = stopAtLimitSwitch;
    this.elevator = elevator;
    this.desired = desired;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.elevatorOn(desired);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //conditional
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (stopAtLimitSwitch) {
      return elevator.getLimitSwitch() || 
          !(elevator.Motor1IsSafe() || elevator.Motor2IsSafe()); //and?
    } else {
      return false;
    }
  }
}
