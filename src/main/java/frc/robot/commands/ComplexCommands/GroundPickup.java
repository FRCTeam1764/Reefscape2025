// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.BasicCommands.ElevatorCommand;
import frc.robot.commands.BasicCommands.IntakeCommand;
import frc.robot.commands.BasicCommands.WristCommand;
import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.IntakeWrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GroundPickup extends SequentialCommandGroup {
  /** Creates a new GroundPickup. */
  public GroundPickup(IntakeWrist wrist, IntakeRollers roller, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  
    SequentialCommandGroup position = new SequentialCommandGroup(
      new ElevatorCommand(elevator, CommandConstants.ELEVATOR_STOP_SAFE, false),
      new WristCommand(wrist, CommandConstants.WRIST_HALF),
      new ParallelCommandGroup(
        new WristCommand(wrist, CommandConstants.WRIST_GRAB_ANGLE),
        new ElevatorCommand(elevator, 0, true)
      )
    );

    addCommands(position, new IntakeCommand(roller, 0.2));
    CommandConstants.ALGAE = true;
  } 
}