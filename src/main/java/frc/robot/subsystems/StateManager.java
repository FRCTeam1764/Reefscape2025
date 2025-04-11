// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Micro;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.estimator.SteadyStateKalmanFilter;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.state.ALGAE_KNOCK_HIGH;
import frc.robot.state.BARGE;
import frc.robot.state.ALGAE_KNOCK_LOW;
import frc.robot.state.BasicState;
import frc.robot.state.IDLE;
import frc.robot.state.IDLE_ALGAE;
import frc.robot.state.IDLE_CORAL;
import frc.robot.state.INTAKE_ALGAE_GROUND;
import frc.robot.state.INTAKE_ALGAE_HIGH;
import frc.robot.state.INTAKE_ALGAE_LOW;
import frc.robot.state.INTERPOLATED_STATE;
import frc.robot.state.INTAKE_CORAL;
import frc.robot.state.INTAKE_CORAL_GROUND;
import frc.robot.state.L1;
import frc.robot.state.L2;
import frc.robot.state.L3;
import frc.robot.state.PROCESSOR;
import frc.robot.state.L4;

public class StateManager extends SubsystemBase {

  Map<String, Object> desiredData = new HashMap<>();
  Map<String, Object> currentData = new HashMap<>();
  boolean willScore = true;
  public States desiredButtonState = States.IDLE;

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
    INTAKE_ALGAE_GROUND,
    ALGAE_KNOCK_LOW,
    ALGAE_KNOCK_HIGH,
    INTAKE_ALGAE_LOW,
    INTAKE_ALGAE_HIGH,
    SPIT_OUT,
    INTERPOLATED_STATE,
    INTAKE_CORAL_GROUND
  }

  public List<BasicState> StateHandlers = List.of(
      new L1(),
      new L2(),
      new L3(),
      new L4(),
      new ALGAE_KNOCK_HIGH(),
      new ALGAE_KNOCK_LOW(),
      new BARGE(),
      new IDLE_ALGAE(),
      new IDLE_CORAL(),
      new IDLE(),
      new INTAKE_ALGAE_GROUND(),
      new INTAKE_ALGAE_HIGH(),
      new INTAKE_ALGAE_LOW(),
      new INTAKE_CORAL(),
      new INTERPOLATED_STATE(),
      new PROCESSOR(),
      new INTAKE_CORAL_GROUND()

  );

  public States state;

  /** Creates a new StateManager. */
  public StateManager() {
    

  }

  public void requestNewState(States newstate) {
    for (BasicState handler : StateHandlers) {
      if (handler.matches(newstate)) {
        handler.execute(this);
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

  



  public void returnToIdle() {
    if ((boolean) currentData.get("IntakeLimitSwitch")) {
      if (state == States.INTAKE_CORAL) {
        requestNewState(States.IDLE_CORAL);
      } else if (state == States.INTAKE_ALGAE_GROUND || state == States.INTAKE_ALGAE_LOW
          || state == States.INTAKE_ALGAE_HIGH) {
        requestNewState(States.IDLE_ALGAE);
      }else{
        requestNewState(States.IDLE_ALGAE); //this should not happen

      }
    } else {
      requestNewState(States.IDLE);
    }
  }

  public void setWillScore(boolean score) {
    willScore = score;
  }

  public boolean getWillScore() {
    return willScore;
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
  @Override
  public void periodic() {

    for (Map.Entry<String, Object> entry : desiredData.entrySet()) {
      String key = entry.getKey();
      Object value = entry.getValue();

      if (value instanceof Double) {
        SmartDashboard.putNumber(key, (Double) value);
      }else if (value instanceof Boolean) {
        SmartDashboard.putBoolean(key, (Boolean) value);
      }else if (value instanceof Integer ){
        SmartDashboard.putNumber(key, (Integer) value);
      }
  }

    // This method will be called once per scheduler run
  }
}