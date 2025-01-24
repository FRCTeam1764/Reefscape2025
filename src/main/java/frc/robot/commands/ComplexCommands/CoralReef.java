// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import java.util.concurrent.locks.Condition;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BasicCommands.ElevatorCommand;
import frc.robot.commands.BasicCommands.ElevatorDownCommand;
import frc.robot.commands.BasicCommands.IntakeCommand;
import frc.robot.commands.BasicCommands.WristCommand;
import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralReef extends SequentialCommandGroup {
  /** Creates a new CoralReef. */
  public CoralReef(ElevatorSubsystem elevator, IntakeSubsystem intake, int level) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements();

    SequentialCommandGroup goToPosition = new SequentialCommandGroup(
      new ElevatorCommand(elevator, elevator.retriveEncoder(level)), 
      new WristCommand(intake, CommandConstants.INTAKE_REEF_ENCODERVALUE)
    );

    ParallelDeadlineGroup spitOut = new ParallelDeadlineGroup(
      new WaitCommand(5), 
      new IntakeCommand(intake, CommandConstants.INTAKE_CORAL_OUT_SPEED)
    );

    SequentialCommandGroup goBack = new SequentialCommandGroup(
      new WristCommand(intake, CommandConstants.INTAKE_REEF_ENCODERVALUE),
      new ElevatorDownCommand(elevator)
    );

    addCommands(goToPosition, spitOut, goBack);
  }
}
