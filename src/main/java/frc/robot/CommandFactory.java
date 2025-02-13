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
import frc.robot.commands.BasicCommands.ElevatorCommand;
import frc.robot.commands.BasicCommands.IntakeCommand;
import frc.robot.commands.ComplexCommands.returnToIdle;
import frc.robot.commands.DriveCommands.DriveToTarget;
import frc.robot.commands.DriveCommands.LockOnAprilTag;
import frc.robot.constants.CommandConstants;
import frc.robot.state.ALGAE_KNOCK_HIGH;
import frc.robot.state.ALGAE_KNOCK_LOW;
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

public class CommandFactory {

    public enum desiredAction {
        ALGAE_KNOCK_HIGH,
        ALGAE_KNOCK_LOW,
        BARGE,
        INTAKE_ALGAE_GROUND,
        INTAKE_ALGAE_HIGH,
        INTAKE_ALGAE_LOW,
        INTAKE_CORAL,
        SCOREL4,
        SCOREL3,
        SCOREL2,
        SCOREL1,
        PROCESSOR
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
    private States[] stateList = { States.L4, States.L3, States.L2, States.L1 };
    private Joystick driver;

    public CommandFactory(Climber climber, Elevator elevator, IntakeRollers intakeRollers, IntakeWrist intakeWrist,
            LimelightSubsystem Limelight4, LimelightSubsystem Limelight3, LimelightSubsystem Limelight2,
            Joystick driver, SwerveSubsystem swerve, StateManager stateManager) {
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
                new RequestStateChange(
                        index == 0 ? States.L1 : index == 1 ? States.L2 : index == 2 ? States.L3 : States.L4,
                        stateManager),
                new ParallelCommandGroup(
                        new DriveToTarget(swerve, Limelight2),
                        new waitUntilPosition(stateManager, CommandConstants.INTAKE_KEY, 4,
                                CommandConstants.ELEVATOR_KEY, 4))).finallyDo((key) -> interupted(key));
    }

    private Command LevelScore() {
        return new SequentialCommandGroup(
                new IntakeCommand(intakeRollers, CommandConstants.INTAKE_CORAL_OUT_SPEED, true),
                new returnToIdle(stateManager, States.IDLE_CORAL)).finallyDo((key) -> interupted(key));
    }

    // AUTOMATED COMMANDS

    private Command AlgaeKnock(boolean high) {
        return new SequentialCommandGroup(
                new RequestStateChange(high ? States.ALGAE_KNOCK_HIGH : States.ALGAE_KNOCK_LOW, stateManager),
                new waitUntilPosition(stateManager, CommandConstants.INTAKE_KEY, 4, CommandConstants.ELEVATOR_KEY, 4),
                new returnToIdle(stateManager, States.IDLE)).finallyDo((key) -> interupted(key));
    }

    private Command Barge() {
        return new SequentialCommandGroup(
                new RequestStateChange(States.BARGE, stateManager),
                new IntakeCommand(intakeRollers, CommandConstants.ALGAE_BARGE_OUT_SPEED, false),
                new WaitUntilCommand(() -> !intakeRollers.getIntakeLimitSwitch())).finallyDo((key) -> interupted(key));
    }

    private Command AlgaeGround() {
        return new SequentialCommandGroup(
                new RequestStateChange(States.INTAKE_ALGAE_GROUND, stateManager),
                new waitUntilPosition(stateManager, CommandConstants.INTAKE_KEY, 4, CommandConstants.ELEVATOR_KEY, 4),
                new IntakeCommand(intakeRollers, CommandConstants.INTAKE_GRAB_ALGAE_SPEED, true),
                new returnToIdle(stateManager, States.IDLE_ALGAE)).finallyDo((key) -> interupted(key));
    }

    private Command AlgaeReefIntake(boolean low) {
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> leftLimelight ? Limelight4.hasTarget() : Limelight3.hasTarget()),
                new RequestStateChange(low ? States.INTAKE_ALGAE_LOW : States.INTAKE_ALGAE_HIGH, stateManager),
                new ParallelCommandGroup(
                        new LockOnAprilTag(swerve, Limelight2, 0, driver, false),
                        new waitUntilPosition(stateManager, CommandConstants.INTAKE_KEY, 4,
                                CommandConstants.ELEVATOR_KEY, 4)),
                new IntakeCommand(intakeRollers, CommandConstants.INTAKE_GRAB_ALGAE_SPEED, true),
                new returnToIdle(stateManager, States.IDLE_ALGAE)).finallyDo((key) -> interupted(key));
    }

    private Command IntakeCoral() {
        return new SequentialCommandGroup(
            new RequestStateChange(States.INTAKE_CORAL, stateManager),
            new WaitUntilCommand(() -> intakeRollers.getIntakeLimitSwitch()),
            new ParallelDeadlineGroup( new WaitUntilCommand(() -> !intakeRollers.getIntakeLimitSwitch()), 
            new ElevatorCommand(elevator, 2, true) //TODO: GET PROPER ANGLE
            )
        ).finallyDo((key) -> interupted(key))
        
        ;
    }

    private Command Level(int index) {
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> leftLimelight ? Limelight4.hasTarget() : Limelight3.hasTarget()),
                new RequestStateChange(
                        index == 0 ? States.L1 : index == 1 ? States.L2 : index == 2 ? States.L3 : States.L4,
                        stateManager),
                new ParallelCommandGroup(
                        new DriveToTarget(swerve, Limelight2),
                        new waitUntilPosition(stateManager, CommandConstants.INTAKE_KEY, 4,
                                CommandConstants.ELEVATOR_KEY, 4)),
                new IntakeCommand(intakeRollers, CommandConstants.INTAKE_CORAL_OUT_SPEED, true),
                new returnToIdle(stateManager, States.IDLE_CORAL)).finallyDo((key) -> interupted(key));
    }

    private Command AlgaeProcessor() {
        return new SequentialCommandGroup(
                new RequestStateChange(States.INTAKE_ALGAE_GROUND, stateManager),
                new waitUntilPosition(stateManager, CommandConstants.INTAKE_KEY, 4, CommandConstants.ELEVATOR_KEY, 4),
                new ParallelDeadlineGroup(new WaitUntilCommand(() -> !intakeRollers.getIntakeLimitSwitch()),
                        new IntakeCommand(intakeRollers, CommandConstants.ALGAE_OUT_SPEED, false)),

                new returnToIdle(stateManager, States.INTAKE_ALGAE_GROUND)).finallyDo((key) -> interupted(key));
    }

    public Command interupted(boolean wasInteruppted) {
        if (wasInteruppted) {
            return new InstantCommand();
        }

        return new returnToIdle(stateManager);
    }

    public Command getDesiredAction() {
        return getAction(currentAction);
    }

    public Command getAction(desiredAction action) {

        switch (action) {
            case ALGAE_KNOCK_HIGH:
                return AlgaeKnock(true);
            case ALGAE_KNOCK_LOW:
                return AlgaeKnock(false);
            case BARGE:
                return Barge();
            case INTAKE_ALGAE_GROUND:
                return AlgaeGround();
            case INTAKE_ALGAE_HIGH:
                return AlgaeReefIntake(false);
            case INTAKE_ALGAE_LOW:
                return AlgaeReefIntake(true);
            case INTAKE_CORAL:
                return IntakeCoral();
            case SCOREL4: // you didn't have to make it start at 0 aleyah that makes it make less sense
                return Level(3);
            case SCOREL3:
                return Level(2);
            case SCOREL2:
                return Level(1);
            case SCOREL1:
                return Level(0);
            case PROCESSOR:
                return AlgaeProcessor();
            default:
                return new InstantCommand(); // EQUIVALNT TO NULL, CHECK LATER TODO:
        }
    }

    public void setDesiredAction(desiredAction currAction) {
        currentAction = currAction;
    }
}
