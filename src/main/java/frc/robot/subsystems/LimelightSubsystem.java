// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  private static NetworkTable table;
  public double xOffset;
  public double isThereTarget;
  private double move;
  
  // finds the distance from the crosshair to target on the x axis
  public LimelightSubsystem(NetworkTable table) {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    xOffset = table.getEntry("tx").getDouble(0);
    
  }

  public void setPipeline(int pipeline) {
		NetworkTableEntry pipelineEntry = table.getEntry("pipeline");
    pipelineEntry.setNumber(pipeline);
  }

  // checks if there is a target in sights 
  public double updateIsThereTarget() {
    isThereTarget = table.getEntry("tv").getDouble(0); 


    return isThereTarget;
  }

  // turns the x offset into how much to move the robot
  public double whereToMove() {
    move = table.getEntry("tx").getDouble(0) * -0.1;
    return move;
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
