// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.BasicCommands.RequestStateChange;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

public class cheaterDPAD {
  /** Creates a new cheaterDPAD. */
  private CommandFactory factory;
  private StateManager stateManager;
  private boolean coralMode = true;
  private String action = "L2";
  private String mode = "Coral";

  public cheaterDPAD(CommandFactory factory, StateManager stateManager) {
    this.factory = factory;
    this.stateManager = stateManager;
  }

  public void algeeMode() {
    coralMode= false;
  }

  public void coralMode() {
    coralMode= true;
  }


  public Command fetchUpPress() {
    if (coralMode) {
      return factory.Level4Position();
    } else {
      return factory.algaeHighPosition();
    }
  }
  public Command fetchUpRelease() {
    if (coralMode) {
      return factory.Level4Score();
    } else {
      return new RequestStateChange(States.IDLE_ALGAE, stateManager);
    }
  }

  public Command fetchRightPress() {
    if (coralMode) {
      return factory.LevelPosition(3);
    } else {
      return factory.algaeLowPosition();
    }
  }

  public Command fetchRightRelease() {
    if (coralMode) {
      return factory.LevelScore();
    } else {
      return new RequestStateChange(States.IDLE_ALGAE, stateManager);
    }
  }

  public Command fetchDownPress() {
    if (coralMode) {
      return factory.LevelPosition(2);
    } else {
      return new RequestStateChange(States.INTAKE_ALGAE_GROUND, stateManager);
    }
  }

  public Command fetchDownRelease() {
    if (coralMode) {
      return factory.LevelScore();
    } else {
      return new RequestStateChange(States.IDLE_ALGAE, stateManager);
    }
  }

  public Command fetchLeftPress() {
    if (coralMode) {
      return factory.LevelPosition(1);
    } else {
      return factory.algaeProcessorPosition();
    }
  }

  public Command fetchLeftRelease() {
    if (coralMode) {
      return factory.LevelScore();
    } else {
      return factory.algaeProcessorScore();
    }
  }

  public void set(String wantedMode, String wantedAction) {
    action = wantedAction;
    mode = wantedMode;
  }


}
