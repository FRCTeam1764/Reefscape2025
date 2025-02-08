// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state.BasicState;
import frc.robot.state.L1;
import frc.robot.state.L2;
import frc.robot.state.L3;
import frc.robot.state.L4;

public class StateManager extends SubsystemBase {

  Map<String, Object> desiredData = new HashMap<>();
  Map<String, Object> currentData = new HashMap<>();
  boolean isAtLocation = false;

  // TODO: FIGURE OUT IF WE NEED A "SCORE" STATE FOR ALL POSITIONS
  public enum States {
    IDLE,
    IDLE_CORAL,
    IDLE_ALGAE,
    L4,
    L3,
    L2,
    L1,
    PROCESSOR,
    BARGE,
    INTAKE_CORAL,
    INTAKE_ALGAE_PREP,
    INTAKE_ALGAE_GROUND,
    INTAKE_ALGAE_LOW,
    INTAKE_ALGAE_HIGH,
    INTERPOLATED_STATE // to ensure things don't go wrong

    // L4_SCORE,
    // L3_SCORE,
    // L2_SCORE,
    // L1_SCORE,
    // PROCESSOR_SCORE,
    // BARGE_SCORE,
  }

  public List<BasicState> StateHandlers = List.of(
      new L1(),
      new L2(),
      new L3(),
      new L4()

  );

  public States state;

  /** Creates a new StateManager. */
  public StateManager() {
    DataLogManager.start();
    

  }

  public void requestNewState(States newstate) {
    for (BasicState handler : StateHandlers) {
      if (handler.matches(newstate)) {
        handler.execute(this);
        isAtLocation = false;
        this.state = newstate;
      }
    }
  }

  public void clearCommandData() {

  }

  public Object getCurrentData(String key) {
    Object value = currentData.get(key);

    if (value instanceof String) {
      return (String) value;
    } else if (value instanceof Integer) {
      return (Integer) value;
    } else if (value instanceof Double) {
      return (Double) value;
    } else if (value instanceof Boolean) {
      return (Boolean) value;
    } else {
      return null; // Handle cases where the value is not of expected type
    }
  }

  

  public void returnToIdle(States perviousState) {
    if ((boolean) currentData.get("IntakeLimitSwitch")) {
      if (perviousState == States.INTAKE_CORAL) {
        requestNewState(States.IDLE_CORAL);
      } else if (perviousState == States.INTAKE_ALGAE_GROUND || perviousState == States.INTAKE_ALGAE_LOW
          || perviousState == States.INTAKE_ALGAE_HIGH) {
        requestNewState(States.IDLE_ALGAE);
      }
    } else {
      requestNewState(States.IDLE);
    }
  }

  public void clearDesiredData(){
    desiredData.clear();
  }

  public Object getDesiredData(String key) {
    Object value = desiredData.get(key);

    if (value instanceof String) {
      return (String) value;
    } else if (value instanceof Integer) {
      return (Integer) value;
    } else if (value instanceof Double) {
      return (Double) value;
    } else if (value instanceof Boolean) {
      return (Boolean) value;
    } else {
      return null; // Handle cases where the value is not of expected type
    }
  }

  public void removeDesiredData(String key) {
    desiredData.remove(key);
  }

  public void addDesiredData(String key, Object object) {
    desiredData.put(key, object);
  }

  public void updateCurrentData(String key, Object object) {
    currentData.put(key, object);
  }

  public void setState(States state) {
    this.state = state;
  }

  public boolean getisAtLocaiton(){
    return isAtLocation;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}