package frc.robot.state;

import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

public class L1 implements BasicState {
public boolean matches(States state){

    return state.equals(States.L1);
}

public void execute(StateManager stateManager){
    stateManager.clearDesiredData();
//50, 1
    stateManager.addDesiredData(CommandConstants.INTAKE_KEY, 140.0);
    stateManager.addDesiredData(CommandConstants.ELEVATOR_KEY, 15.0);
}
   
}
