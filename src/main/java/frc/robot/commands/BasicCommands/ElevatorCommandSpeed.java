// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.IntakeWristRev;
import frc.robot.subsystems.StateManager;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommandSpeed extends Command {
  /** Creates a new ElevatorCommand. */
  Elevator elevator;
  double speed;
  boolean stopAtLimitSwitch;
  //StateManager stateManager = new StateManager();
  IntakeRollers rollers;
  IntakeWristRev wrist;
  public ElevatorCommandSpeed(Elevator elevator, double speed, IntakeRollers rollers, IntakeWristRev wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    //this.stopAtLimitSwitch = stopAtLimitSwitch;
    this.elevator = elevator;
    this.speed = speed;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.elevatorOnSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //conditional
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
      return elevator.getLimitSwitches();
   
  }
}
