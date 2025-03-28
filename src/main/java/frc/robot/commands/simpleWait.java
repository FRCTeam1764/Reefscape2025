// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class simpleWait extends Command {
  /** Creates a new simpleWaitCommand. */
  double time;
  Timer timer;

  public simpleWait(double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.time = time;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.

  @Override
  public void initialize() {
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.get()>time){
      return true;
    }
    return false;
  }
}
