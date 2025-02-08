// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.waitUntilPosition;
import frc.robot.commands.BasicCommands.RequestStateChange;
import frc.robot.commands.BasicCommands.IntakeCommand;
import frc.robot.commands.ComplexCommands.returnToIdle;
import frc.robot.commands.DriveCommands.DriveToTarget;
import frc.robot.commands.DriveCommands.LockOnAprilTag;
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
    private boolean leftLimelight;
    private Climber climber;
    private Elevator elevator;
    private IntakeRollers intakeRollers;
    private IntakeWrist intakeWrist;
    private LimelightSubsystem Limelight4;
    private LimelightSubsystem Limelight3;
    private LimelightSubsystem Limelight2;
    private SwerveSubsystem swerve;
    private StateManager stateManager;
    private States[] stateList = {States.L4, States.L3, States.L2, States.L1};
    private Joystick driver;



    public CommandFactory(Climber climber, Elevator elevator, IntakeRollers intakeRollers, IntakeWrist intakeWrist,LimelightSubsystem Limelight4, LimelightSubsystem Limelight3, LimelightSubsystem Limelight2, Joystick driver, SwerveSubsystem swerve, StateManager stateManager){
        this.climber = climber;
        this.elevator = elevator;
        this.intakeRollers = intakeRollers;
        this.intakeWrist = intakeWrist;
        this.Limelight4 = Limelight4;
        this.Limelight3 = Limelight3;
        this.Limelight2 = Limelight2;
        this.driver = driver;
        this.swerve = swerve;
        this.stateManager = stateManager;
    }

    public void changeLimelightOrienation(boolean leftLimelight) {
        this.leftLimelight = leftLimelight;
    }

    // TESTING COMMANDS

    private Command LevelPosition(int index) {
        return new SequentialCommandGroup(
            new RequestStateChange(index == 0 ? States.L1 : index == 1 ? States.L2 : index == 2 ? States.L3 : States.L4, stateManager),
            new ParallelCommandGroup(
                new DriveToTarget(swerve, Limelight2),
                new waitUntilPosition(stateManager, CommandConstants.INTAKE_KEY, 4, CommandConstants.ELEVATOR_KEY, 4)
            )
        );
    }

    private Command LevelScore() {
        return new SequentialCommandGroup(
            new IntakeCommand(intakeRollers, CommandConstants.INTAKE_CORAL_OUT_SPEED),
            new WaitCommand(1),
            new returnToIdle(stateManager, States.IDLE)
        );
    }


    // AUTOMATED COMMANDS

    private Command Level(int index) {
        return new SequentialCommandGroup(
            new WaitUntilCommand(() -> leftLimelight ? Limelight4.hasTarget() : Limelight3.hasTarget()),
            new RequestStateChange(index == 0 ? States.L1 : index == 1 ? States.L2 : index == 2 ? States.L3 : States.L4, stateManager),
            new ParallelCommandGroup(
                new DriveToTarget(swerve, Limelight2),
                new waitUntilPosition(stateManager, CommandConstants.INTAKE_KEY, 4, CommandConstants.ELEVATOR_KEY, 4)
            ),
            new IntakeCommand(intakeRollers, CommandConstants.INTAKE_CORAL_OUT_SPEED),
            new WaitUntilCommand(() -> intakeRollers.getIntakeBreakbeam()),
            new returnToIdle(stateManager, States.IDLE_CORAL)
        );
    }

    private Command algaeReefIntake(int index) { //idle algae
        return new SequentialCommandGroup(
            new WaitUntilCommand(() -> leftLimelight ? Limelight4.hasTarget() : Limelight3.hasTarget()),
            new RequestStateChange(index == 0 ? States.INTAKE_ALGAE_LOW : index == 1 ? States.INTAKE_ALGAE_LOW : States.INTAKE_ALGAE_HIGH, stateManager),
            new ParallelCommandGroup(
                new LockOnAprilTag(swerve, Limelight2, 0, driver, false),
                new waitUntilPosition(stateManager, CommandConstants.INTAKE_KEY, 4, CommandConstants.ELEVATOR_KEY, 4)
            ),
            new IntakeCommand(intakeRollers, CommandConstants.INTAKE_GRAB_ALGAE),
            new WaitUntilCommand(() -> intakeRollers.getIntakeBreakbeam()),
            new returnToIdle(stateManager, States.IDLE_ALGAE)
        );
    }

    //TODO: ADD MORE CASES/STATES
    public Command getDesiredAction() {
    switch (currentAction) {
        case SCOREL4:
            return Level(3);
        case SCOREL3:
            return Level(2);
        case SCOREL2:
            return Level(1);
        case SCOREL1:
            return Level(0);
        default:
            return new InstantCommand(); //EQUIVALNT TO NULL, CHECK LATER TODO:
        }
    }

    public void setDesiredAction(desiredAction currAction) {
        currentAction = currAction;
    }
}
