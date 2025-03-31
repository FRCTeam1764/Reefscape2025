package frc.robot.state;

import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

public class BARGE implements BasicState {
public boolean matches(States state){

    return state.equals(States.BARGE);
}

public void execute(StateManager stateManager){
    stateManager.clearDesiredData();

    stateManager.addDesiredData(CommandConstants.ROLLER_KEY, -0.1);
    stateManager.addDesiredData(CommandConstants.INTAKE_KEY, 40.0);
    stateManager.addDesiredData(CommandConstants.ELEVATOR_KEY, 24.7);
}
   
}
