// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.BasicCommands.ElevatorCommand;
import frc.robot.commands.BasicCommands.WristCommand;
import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoBackDefault extends SequentialCommandGroup {
  /** Creates a new GoBackDefault. */
  public GoBackDefault(IntakeSubsystem intake, ElevatorSubsystem elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    SequentialCommandGroup empty = new SequentialCommandGroup(
      new WristCommand(intake, CommandConstants.WRIST_DOWN), 
      new ElevatorCommand(elevator, 0, true)
    );

    ParallelCommandGroup down = new ParallelCommandGroup(
      new ElevatorCommand(elevator, 0, true),
      new WristCommand(intake, CommandConstants.WRIST_ALGAE)
    );

    SequentialCommandGroup algae = new SequentialCommandGroup(
      // elevator.getEncoderValue() >= CommandConstants.ELEVATOR_STOP_SAFE //is elevator at a safe spot for wrist movement
      // intake.getEncoderPos() >= CommandConstants.WRIST_HALF // is wrist at least higher than half
      
      new WristCommand(intake, CommandConstants.WRIST_ALGAE),
      new ElevatorCommand(elevator, 0, true)
    );

    // addCommands(
    //   CommandConstants.ALGAE ? algae : empty
    // );

    ParallelCommandGroup wristHalf = new ParallelCommandGroup(
      new ElevatorCommand(elevator, 0, true),
      new WristCommand(intake, CommandConstants.WRIST_DOWN)
    );

    SequentialCommandGroup wristUp = new SequentialCommandGroup(
      new WristCommand(intake, CommandConstants.WRIST_HALF),
      new ParallelCommandGroup(
        new ElevatorCommand(elevator, 0, true)
      )
    );
    
    addCommands(
      intake.getEncoderPos()<=CommandConstants.WRIST_HALF ? wristHalf : 
        intake.getEncoderPos()>CommandConstants.WRIST_HALF ? wristUp : wristUp
    );
  }
}
