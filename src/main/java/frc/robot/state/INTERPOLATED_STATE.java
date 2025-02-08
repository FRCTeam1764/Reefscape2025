package frc.robot.state;

import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

public class INTERPOLATED_STATE implements BasicState{
public boolean matches(States state){
    return state.equals(States.INTERPOLATED_STATE);
}

public void execute(StateManager stateManager){
    stateManager.clearDesiredData();
    stateManager.addDesiredData(CommandConstants.INTAKE_KEY, 5);
    stateManager.addDesiredData(CommandConstants.ELEVATOR_KEY, 5);
}
}
