package frc.robot.state;

import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

public class L4 implements BasicState {

public boolean matches(States state){

    return state.equals(States.L4);
}

public void execute(StateManager stateManager){
    stateManager.clearDesiredData();

   stateManager.addDesiredData(CommandConstants.INTAKE_KEY, 50);
    stateManager.addDesiredData(CommandConstants.ELEVATOR_KEY, 50);

}

}
