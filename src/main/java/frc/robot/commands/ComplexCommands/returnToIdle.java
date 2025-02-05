// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.waitUntilPosition;
import frc.robot.commands.BasicCommands.RequestStateChange;
import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class returnToIdle extends SequentialCommandGroup {
  /** Creates a new returnToIdle. */
  StateManager stateManager;
  public returnToIdle(StateManager stateManager, States state) {
    this.stateManager = stateManager;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      
    new RequestStateChange(States.INTERPOLATED_STATE, stateManager),
    new waitUntilPosition(stateManager,CommandConstants.INTAKE_KEY, 3, CommandConstants.ELEVATOR_KEY, 1),
    new InstantCommand (() -> stateManager.returnToIdle(state)) 
    );
  }
}
