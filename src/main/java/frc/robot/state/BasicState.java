package frc.robot.state;

import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

public interface BasicState {

    boolean matches(States state);
    void execute(StateManager state);
} 
