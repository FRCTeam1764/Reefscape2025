package frc.robot.state;

import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

public class INTAKE_CORAL_GROUND implements BasicState{
public boolean matches(States state){
    return state.equals(States.INTAKE_CORAL_GROUND);
}

public void execute(StateManager stateManager){
    stateManager.clearDesiredData();
    stateManager.addDesiredData(CommandConstants.ROLLER_KEY, -.6);
    stateManager.addDesiredData(CommandConstants.ELEVATOR_KEY, 4.5);
    stateManager.addDesiredData(CommandConstants.INTAKE_KEY,150.0);
}
}
