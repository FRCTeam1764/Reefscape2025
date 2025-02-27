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
  private String coralAction = "L2";
  private String algaeAction = "processor";

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

  public Command dpadAction(String button){
    switch (button) {
      case ("pressUp"):
        return coralMode ? factory.Level4Position() : factory.algaeHighPosition();
      case ("releaseUp"):
        return coralMode ? factory.Level4Score() : new RequestStateChange(States.IDLE_ALGAE, stateManager);
      case ("pressRight"):
        return coralMode ? factory.LevelPosition(3) : factory.algaeLowPosition();
      case ("releaseRight"):
        return coralMode ? factory.LevelScore() : new RequestStateChange(States.IDLE_ALGAE, stateManager);
      case ("pressDown"):
        return coralMode ? factory.LevelPosition(2) : new RequestStateChange(States.INTAKE_ALGAE_GROUND, stateManager);
      case ("releaseDown"):
        return coralMode ? factory.LevelScore() : new RequestStateChange(States.IDLE_ALGAE, stateManager);
      case ("pressLeft"):
        return coralMode ? factory.LevelPosition(1) : factory.algaeProcessorPosition();
      case ("releaseLeft"):
        return coralMode ? factory.LevelScore() : factory.algaeProcessorScore();
      default:
        return new RequestStateChange(States.IDLE, stateManager);
    }
  }

  public Command coralInitiate() {
    switch (coralAction) {
      case ("level4"):
        return factory.Level4Position();
      case ("level3"):
        return factory.LevelPosition(3);
      case ("level2"):
        return factory.LevelPosition(2);
      case ("level1"):
        return factory.LevelPosition(1);
      default:
        return factory.LevelPosition(2);
    }
  }

  public Command algaeInitiate() {
    switch (algaeAction) {
      case ("lowposition"):
        return factory.algaeLowPosition();
      case ("highposition"):
        return factory.algaeHighPosition();
      case ("groundpickup"):
        return factory.algaeGroundPosition();
      case ("processor"):
        return factory.algaeProcessorPosition();
      default:
        return factory.algaeProcessorPosition();
    }
  }

  public Command coralRelease(boolean four) {
    return coralAction.contains("4") ? factory.Level4Score() : factory.LevelScore();
  }

  public Command algaeRelease() {
    switch (algaeAction) {
      case ("processor"):
        return factory.algaeProcessorScore();
      default:
        return new RequestStateChange(States.IDLE_ALGAE, stateManager);
    }
  }

  public void dpadActionNotUsed(String button){
    if (coralMode) {
      switch (button) {
        case ("pressUp"):
          coralAction = "level4";
          break;
        case ("pressRight"):
          coralAction = "level3";
          break;
        case ("pressDown"):
          coralAction = "level2";
          break;
        case ("pressLeft"):
          coralAction = "level1";
          break;
        default:
          coralAction = "level2";
          break;
      }
    } else {
      switch (button) {
        case ("pressUp"):
          algaeAction = "highposition";
          break;
        case ("pressRight"):
          algaeAction = "lowposition";
          break;
        case ("pressDown"):
          algaeAction = "groundpickup";
          break;
        case ("pressLeft"):
          algaeAction = "processor";
          break;
        default:
          algaeAction = "processor";
          break;
      }
    }
  }
}