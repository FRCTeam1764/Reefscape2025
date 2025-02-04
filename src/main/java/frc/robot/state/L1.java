package frc.robot.state;

import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

public class L1 implements BasicState {
public boolean matches(States state){

    return state.equals(States.L1);
}

public void execute(StateManager stateManager){

    stateManager.addDesiredData("IntakePosition", 5);
    stateManager.addDesiredData("ElevatorPosition", 5);
}
   
}
