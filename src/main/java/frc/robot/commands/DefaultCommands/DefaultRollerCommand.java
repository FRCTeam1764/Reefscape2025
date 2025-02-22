// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DefaultCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.StateManager;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultRollerCommand extends Command {
  /** Creates a new DefaultRollerCommand. */
  IntakeRollers rollers;
  StateManager stateManager;

  public DefaultRollerCommand(IntakeRollers rollers, StateManager stateManager) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.rollers = rollers;
    this.stateManager = stateManager;
    addRequirements(rollers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(stateManager.getDesiredData(CommandConstants.ROLLER_KEY) != null){
      //if (rollers.getIntakeLimitSwitch()) {
        //rollers.wheelsIntake(CommandConstants.INTAKE_HOLDING_SPEED); //TODO: find slower speed
      //} else {
        rollers.wheelsIntake((double) stateManager.getDesiredData(CommandConstants.ROLLER_KEY));
      //}
      SmartDashboard.putBoolean("elseroller", false);
    }else{
      rollers.wheelsIntake(0);
      SmartDashboard.putBoolean("elseroller", true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
