// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.waitUntilPosition;
import frc.robot.commands.BasicCommands.RequestStateChange;
import frc.robot.commands.BasicCommands.IntakeCommand;
import frc.robot.commands.ComplexCommands.returnToIdle;
import frc.robot.commands.DriveCommands.DriveToTarget;
import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.StateManager.States;

//This class will handle all command handling for drivers



/** Add your docs here. */



public class CommandFactory  {

    public enum desiredAction  {
        SCOREL4,
        SCOREL3,
        SCOREL2,
        SCOREL1,
        INTAKE,


    }






    public desiredAction currentAction;
    private Climber climber;
    private Elevator elevator;
    private IntakeRollers intakeRollers;
    private IntakeWrist intakeWrist;
    private LimelightSubsystem Limelight4;
    private LimelightSubsystem Limelight3;
    private LimelightSubsystem Limelight2;
    private SwerveSubsystem swerve;
    private StateManager stateManager;



public CommandFactory(Climber climber, Elevator elevator, IntakeRollers intakeRollers, IntakeWrist intakeWrist,LimelightSubsystem Limelight4, LimelightSubsystem Limelight3, LimelightSubsystem Limelight2, SwerveSubsystem swerve, StateManager stateManager){
this.climber = climber;
this.elevator = elevator;
this.intakeRollers = intakeRollers;
this.intakeWrist = intakeWrist;
this.Limelight4 = Limelight4;
this.Limelight3 = Limelight3;
this.Limelight2 = Limelight2;
this.swerve = swerve;
this.stateManager = stateManager;
}

// TESTING COMMANDS
private Command L4Position() {
    return new SequentialCommandGroup(
        new RequestStateChange(States.L4, stateManager),
        new ParallelCommandGroup(
            new DriveToTarget(swerve, Limelight2),
            new waitUntilPosition(stateManager, CommandConstants.INTAKE_KEY, 4, CommandConstants.ELEVATOR_KEY, 4)
        )
    );
}

private Command L4Score() {
    return new SequentialCommandGroup(
        new IntakeCommand(intakeRollers, CommandConstants.INTAKE_CORAL_OUT_SPEED)
        
    );
}

// AUTOMATED COMMANDS
//example,unfinished - TODO, IMPLIMENT LIMELIGHT/ CALL THE COMMAND RETURN TO IDLE
private Command L4(){
    return new SequentialCommandGroup(
        new DriveToTarget(swerve, Limelight2), //look at limelights later
        new RequestStateChange(States.L4, stateManager),
        new waitUntilPosition(stateManager, CommandConstants.INTAKE_KEY, 4, CommandConstants.ELEVATOR_KEY, 4)
    );
}

private Command L3() {
    return new InstantCommand();
}

//TODO: ADD MORE CASES/STATES
    public Command getDesiredAction(){
switch (currentAction) {
    case SCOREL4:
        return L4();
    case SCOREL3:
        return L3();

    default:
    return new InstantCommand(); //EQUIVALNT TO NULL, CHECK LATER TODO:
    }


}

public void setDesiredAction(desiredAction currAction){
currentAction = currAction;
}
}
