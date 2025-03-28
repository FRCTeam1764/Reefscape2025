// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ComplexCommands.returnToIdle;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommandLimit extends Command {
  /** Creates a new ElevatorCommand. */
  Elevator elevator;
  boolean stopAtLimitSwitch;
  public ElevatorCommandLimit(Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.elevatorOnSpeed(-0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.elevatorOnSpeed(-0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.elevatorOnSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return  this.elevator.limitSwitchBottom2.get() && this.elevator.limitSwitchBottom1.get();

  }
}
