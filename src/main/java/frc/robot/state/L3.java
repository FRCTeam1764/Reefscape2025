package frc.robot.state;

import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

public class L3 implements BasicState {

public boolean matches(States state){

    return state.equals(States.L3);
}

public void execute(StateManager stateManager){
stateManager.addDesiredData("IntakePosition", 20);
stateManager.addDesiredData("ElevatorPosition", 20);

}

}
