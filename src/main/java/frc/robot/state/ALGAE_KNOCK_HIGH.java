package frc.robot.state;

import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

public class ALGAE_KNOCK_HIGH implements BasicState{
public boolean matches(States state){
    return state.equals(States.ALGAE_KNOCK_HIGH);
}

public void execute(StateManager stateManager){
    stateManager.clearDesiredData();
    stateManager.addDesiredData(CommandConstants.ROLLER_KEY, .5);
    stateManager.addDesiredData(CommandConstants.ELEVATOR_KEY, 15);
    stateManager.addDesiredData(CommandConstants.INTAKE_KEY,90 );
}
}
