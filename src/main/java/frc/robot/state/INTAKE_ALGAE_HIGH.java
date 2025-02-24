package frc.robot.state;

import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

public class INTAKE_ALGAE_HIGH implements BasicState{
public boolean matches(States state){
    return state.equals(States.INTAKE_ALGAE_HIGH);
}

public void execute(StateManager stateManager){
    stateManager.clearDesiredData();
    stateManager.addDesiredData(CommandConstants.ROLLER_KEY, -.5);
    stateManager.addDesiredData(CommandConstants.ELEVATOR_KEY, 11.5);
    stateManager.addDesiredData(CommandConstants.INTAKE_KEY,75.0 );
}
}
