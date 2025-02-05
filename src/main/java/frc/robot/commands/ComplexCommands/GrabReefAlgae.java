// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.BasicCommands.ElevatorCommand;
import frc.robot.commands.BasicCommands.WristCommand;
import frc.robot.commands.BasicCommands.IntakeCommand;
import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.IntakeWrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrabReefAlgae extends ParallelCommandGroup {
  /** Creates a new GrabAlgae. */
  public GrabReefAlgae(Elevator elevator, IntakeRollers rollers, IntakeWrist wrist, int level) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(elevator, rollers, wrist);

    SequentialCommandGroup position = new SequentialCommandGroup(
      new ElevatorCommand(elevator, elevator.retriveLevelEncoder(level), false),
      new WristCommand(wrist, CommandConstants.WRIST_GRAB_ANGLE)
    );

    ParallelCommandGroup grab = new ParallelCommandGroup(
      new WristCommand(wrist, CommandConstants.WRIST_GRAB_ANGLE),
      new IntakeCommand(rollers, CommandConstants.INTAKE_GRAB_ALGAE_OUT)
    );
    
    CommandConstants.ALGAE = true;
    addCommands(position, grab);
  }
}
