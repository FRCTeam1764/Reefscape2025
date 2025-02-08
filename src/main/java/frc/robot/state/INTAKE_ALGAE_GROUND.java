package frc.robot.state;

import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

public class INTAKE_ALGAE_GROUND implements BasicState{
public boolean matches(States state){
    return state.equals(States.INTAKE_ALGAE_GROUND);
}

public void execute(StateManager stateManager){
    stateManager.clearDesiredData();
    stateManager.addDesiredData("", stateManager);
    stateManager.addDesiredData("ElevatorPosition", 5);
    stateManager.addDesiredData("IntakeWristPosition",5 );
}
}
