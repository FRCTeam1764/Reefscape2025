package frc.robot.state;

import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

public class INTAKE_CORAL implements BasicState{
public boolean matches(States state){
    return state.equals(States.INTAKE_CORAL);
}

public void execute(StateManager stateManager){
    stateManager.clearDesiredData();
    stateManager.addDesiredData(CommandConstants.ROLLER_KEY, -.3);
    stateManager.addDesiredData(CommandConstants.ELEVATOR_KEY, 11.0);
    stateManager.addDesiredData(CommandConstants.INTAKE_KEY,195.0 );
}
}
