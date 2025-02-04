// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.BasicCommands.ElevatorCommand;
import frc.robot.commands.BasicCommands.WristCommand;
import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeWrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BounceReefAlgae extends SequentialCommandGroup {
  /** Creates a new TakeAlgae. */
  public BounceReefAlgae(Elevator elevator, IntakeWrist intake, int level) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(intake, elevator);

    SequentialCommandGroup position = new SequentialCommandGroup(
      new ElevatorCommand(elevator, elevator.retriveLevelEncoder(level), false),
      new WristCommand(intake, CommandConstants.WRIST_DOWN)
    );

    Command getAlgaeOut = new WristCommand(intake, CommandConstants.INTAKE_BOUNCE_ALGAE_OUT);

    

    addCommands(position, getAlgaeOut, new WristCommand(intake, CommandConstants.WRIST_UP));
  }
}
