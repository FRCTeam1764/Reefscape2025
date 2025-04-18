package frc.robot.state;

import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

public class IDLE_ALGAE implements BasicState {
public boolean matches(States state){

    return state.equals(States.IDLE_ALGAE);
}

public void execute(StateManager stateManager){
    stateManager.clearDesiredData();

    stateManager.addDesiredData(CommandConstants.ROLLER_KEY, -0.075);
    stateManager.addDesiredData(CommandConstants.INTAKE_KEY, 25.0); //30
    stateManager.addDesiredData(CommandConstants.ELEVATOR_KEY, 1.0);
}
   
}
