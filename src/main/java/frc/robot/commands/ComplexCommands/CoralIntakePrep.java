// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BasicCommands.ElevatorCommand;
import frc.robot.commands.BasicCommands.IntakeCommand;
import frc.robot.commands.BasicCommands.WristCommand;
import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralIntakePrep extends SequentialCommandGroup {
  /** Creates a new CoralIntakePrep. */
  public CoralIntakePrep(IntakeSubsystem intake, ElevatorSubsystem elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    ParallelDeadlineGroup grab = new ParallelDeadlineGroup(
      new WaitCommand(2), 
      new IntakeCommand(intake, 0.3));
//TODO: NEED TO ENSURE ELAVTOR MOVES DOWN LOW AFTER WRIST IS CLEAR, CURRENTLY WILL NOT 
//IF POSITION IS NOT GREATER THAN 20 
    SequentialCommandGroup up = new SequentialCommandGroup(
      new ElevatorCommand(elevator, CommandConstants.ELEVATOR_STOP_SAFE, false),
      new ParallelCommandGroup(
        new WristCommand(intake, CommandConstants.WRIST_UP),
        new ElevatorCommand(elevator, 0, true)
          .onlyIf(() -> (intake.getEncoderPos()>20)) //TODO: fix half out encoder MOVE NUMBER INTO CONSTANTS
      )    
    );
    
    addCommands(
      grab,
      up
    );
  }
}
