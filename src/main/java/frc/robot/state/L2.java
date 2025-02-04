package frc.robot.state;

import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

public class L2 implements BasicState {
public boolean matches(States state){

    return state.equals(States.L2);
}

public void execute(StateManager stateManager){

    stateManager.addDesiredData("IntakePosition", 10);
    stateManager.addDesiredData("ElevatorPosition", 10);
}
   
}
