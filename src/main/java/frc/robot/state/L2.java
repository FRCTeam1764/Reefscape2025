package frc.robot.state;

import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

public class L2 implements BasicState {
public boolean matches(States state){

    return state.equals(States.L2);
}

public void execute(StateManager stateManager){
    stateManager.clearDesiredData();

    stateManager.addDesiredData(CommandConstants.INTAKE_KEY, 30.0);
    stateManager.addDesiredData(CommandConstants.ELEVATOR_KEY, 2.5);
}
   
}
