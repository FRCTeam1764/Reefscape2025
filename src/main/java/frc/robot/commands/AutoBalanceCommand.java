// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.SwerveConstants;
import frc.robot.state.RobotState;
import frc.robot.subsystems.Swerve;

public class AutoBalanceCommand extends CommandBase {
public Swerve s_Swerve;
public boolean robotCentric;
public RobotState robotState;


  /** Creates a new AutoBalanceCommand. */
  public AutoBalanceCommand(Swerve s_Swerve,RobotState robotState, boolean robotCentric) {
    
    // Use addRequirements() here to declare subsystem dependencies.

    this.s_Swerve = s_Swerve;
    this.robotState = robotState;
    this.robotCentric = robotCentric;


    addRequirements(s_Swerve);
  }



double error;
double autoLevelPwr;
public double getAutoLevel(){
    error = -s_Swerve.getNavx().getPitch();
    if(Math.abs(error)<1){
        robotState.swerveState.noSwerveAutoBalance();;
    }
    autoLevelPwr = -Math.min(error*.015 , 1); //previously was .018
    // System.out.println(autoLevelPwr);
    return autoLevelPwr;

}




  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotState.swerveState.swerveAutoBalance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
// System.out.println("it be working");
    if (robotState.swerveState.getSwerveState()){
s_Swerve.drive(new Translation2d( -getAutoLevel(),0).times(SwerveConstants.Swerve.maxSpeed),
  0, 
  !robotCentric,
  true
);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !robotState.swerveState.getSwerveState();
  }
}
