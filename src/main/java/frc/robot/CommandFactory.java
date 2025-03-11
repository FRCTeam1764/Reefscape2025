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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.waitUntilPosition;
import frc.robot.commands.waitUntilPositionIndex;
import frc.robot.commands.BasicCommands.RequestStateChange;
import frc.robot.commands.BasicCommands.WristCommand;
import frc.robot.commands.BasicCommands.ClimberCommand;
import frc.robot.commands.BasicCommands.ElevatorCommand;
import frc.robot.commands.BasicCommands.IntakeCommand;
import frc.robot.commands.ComplexCommands.returnToIdle;
import frc.robot.commands.DriveCommands.DriveToTarget;
import frc.robot.commands.DriveCommands.DriveToTargetOffset;
import frc.robot.commands.DriveCommands.LockOnAprilTag;
import frc.robot.commands.DriveCommands.TurnToAngle;
import frc.robot.constants.CommandConstants;
import frc.robot.state.ALGAE_KNOCK_HIGH;
import frc.robot.state.ALGAE_KNOCK_LOW;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.IntakeWristRev;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

//This class will handle all command handling for drivers

/** Add your docs here. */

public class CommandFactory {

    public enum desiredAction {
        ALGAE_KNOCK_HIGH,
        ALGAE_KNOCK_LOW,
        BARGE,
        CLIMBER,
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

    public desiredAction currentAction = desiredAction.SCOREL2;
    private boolean leftLimelight;
    //private Climber climber;
    private Elevator elevator;
    private IntakeRollers intakeRollers;
    private IntakeWristRev intakeWrist;
    private LimelightSubsystem Limelight4;
    private LimelightSubsystem Limelight3;
    private LimelightSubsystem Limelight2;
    private CommandSwerveDrivetrain swerve;
    private StateManager stateManager;
    private States[] stateList = { States.L4, States.L3, States.L2, States.L1 };
    private CommandXboxController driver;
    private boolean left = true;

    public CommandFactory( Elevator elevator, IntakeRollers intakeRollers, IntakeWristRev intakeWrist,
            LimelightSubsystem Limelight4, LimelightSubsystem Limelight3, LimelightSubsystem Limelight2,
            CommandXboxController driver, CommandSwerveDrivetrain swerve, StateManager stateManager) {
      //  this.climber = climber;
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

    public Command LevelPosition(int index) {
        return new RequestStateChange(
            index == 1 ? States.L1 : index == 2 ? States.L2 : States.L3, stateManager);
    }

    public Command LevelScore() {
        return new SequentialCommandGroup(
                new WristCommand(intakeWrist, 40),
                new ParallelDeadlineGroup(
                    new WaitCommand(.5),
                    new IntakeCommand(intakeRollers, .2, false).asProxy(),
                    new WristCommand(intakeWrist, 40)).asProxy(),
                new RequestStateChange(States.IDLE, stateManager));
    }

    public Command Level4Position() {
        return new RequestStateChange(States.L4, stateManager);
    }
    
    public Command Level4Score() {
        return new SequentialCommandGroup(
                new WristCommand(intakeWrist, 40),
                new ParallelDeadlineGroup(
                    new WaitCommand(0.5),
                    new IntakeCommand(intakeRollers, .2, false).asProxy(),
                    new WristCommand(intakeWrist, 40).asProxy()),
                new returnToIdle(stateManager)
               );
    }

    public Command algaeGroundPosition() {
        return new RequestStateChange(States.INTAKE_ALGAE_GROUND, stateManager);
    }

    public Command algaeLowPosition() {
        return new RequestStateChange(States.INTAKE_ALGAE_LOW, stateManager);
    }

    public Command algaeHighPosition() {
        return new RequestStateChange(States.INTAKE_ALGAE_HIGH, stateManager);
    }

    public Command algaeIdle() {
        return new RequestStateChange(States.IDLE, stateManager);
    }

    public Command algaeProcessorPosition() {
        return new RequestStateChange(States.PROCESSOR, stateManager);
    }

    

    public Command algaeProcessorScore() {
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                new WaitCommand(1),
                new IntakeCommand(intakeRollers, .3, false)),
            new RequestStateChange(States.IDLE, stateManager)
        );
    }

    public Command IntakeCoralTest() {
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(new WaitCommand(0.5), new ElevatorCommand(elevator, 8.8).asProxy()),
           new RequestStateChange(States.IDLE_CORAL, stateManager),
         
         //   new WaitCommand(1),
            new ParallelDeadlineGroup(new WaitCommand(0.5), new WristCommand(intakeWrist, 60).asProxy()), 
            new RequestStateChange(States.IDLE, stateManager));
    }

    public Command IntakeCoralPosition() {
        return new RequestStateChange(States.INTAKE_CORAL, stateManager);
    }


    //limalight
    public Command LockOnAprilTag() {
        return new LockOnAprilTag(swerve, Limelight3, 0, driver, false);
    }

    public Command TurnToAngle() {
        return new TurnToAngle(swerve, Limelight3);
    }

    public Command DriveToTargetOffset3() {
        return new DriveToTargetOffset(swerve, Limelight3, 0, 0, -15, 15);
    }

    public Command DriveToTargetOffset4() {
        return new DriveToTargetOffset(swerve, Limelight3, 0, 0, -15, 15);
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
                new waitUntilPosition(stateManager, CommandConstants.INTAKE_KEY, 4, CommandConstants.ELEVATOR_KEY, 4),
                new ParallelDeadlineGroup(new WaitCommand(2),
                        new IntakeCommand(intakeRollers, CommandConstants.ALGAE_OUT_SPEED, false)),
                new returnToIdle(stateManager, States.IDLE).finallyDo((key) -> interupted(key)));
    }

    

    private Command AlgaeGround() {
        return new SequentialCommandGroup(
                new RequestStateChange(States.INTAKE_ALGAE_GROUND, stateManager),
                new LockOnAprilTag(swerve, Limelight3, 0, driver, false)).finallyDo((key) -> interupted(key));
    }

    private Command AlgaeReefIntake(boolean low) {
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> leftLimelight ? Limelight4.hasTarget() : Limelight3.hasTarget()),
                new RequestStateChange(low ? States.INTAKE_ALGAE_LOW : States.INTAKE_ALGAE_HIGH, stateManager),
                new ParallelCommandGroup(
                        new LockOnAprilTag(swerve, Limelight2, 0, driver, false),
                        new waitUntilPosition(stateManager, CommandConstants.INTAKE_KEY, 4,
                                CommandConstants.ELEVATOR_KEY, 4)),
                new ParallelDeadlineGroup(
                        new WaitCommand(2),
                        new IntakeCommand(intakeRollers, CommandConstants.INTAKE_GRAB_ALGAE_SPEED, false)),
                new returnToIdle(stateManager, States.IDLE_ALGAE)).finallyDo((key) -> interupted(key));
    }


    public Command interupted(boolean wasInteruppted) {
        if (wasInteruppted) {
            return new InstantCommand();
        }

        return new returnToIdle(stateManager);
    }

    public Command getDesiredAction(boolean press) {
        return press ? getTrueAction(currentAction) : getFalseAction();
    }


    public Command getTrueAction(desiredAction action) {
        switch (action) {
            // case ALGAE_KNOCK_HIGH:
            //     return AlgaeKnock(true);
            // case ALGAE_KNOCK_LOW:
            //     return AlgaeKnock(false);
            // case BARGE:
            //     return Barge();
            // case CLIMBER:
            //     return Climber();
            // case INTAKE_ALGAE_GROUND:
            //     return AlgaeGround();
            case INTAKE_ALGAE_HIGH:
                return AlgaeReefIntake(false);
            case INTAKE_ALGAE_LOW:
                return AlgaeReefIntake(true);
            case INTAKE_CORAL:
                return IntakeCoralPosition();
            case SCOREL4:
                return Level4Position();
            case SCOREL3:
                return LevelPosition(3);
            case SCOREL2:
                return LevelPosition(2);
            case SCOREL1:
                return LevelPosition(1);
            case PROCESSOR:
                return algaeProcessorPosition();
            default:
                return new InstantCommand(); // EQUIVALNT TO NULL, CHECK LATER TODO:
        }
    }

    public Command getFalseAction() {
        switch (currentAction) {
            case INTAKE_ALGAE_HIGH:
                return algaeIdle();
            case INTAKE_ALGAE_LOW:
                return algaeIdle();
            case INTAKE_CORAL:
                return IntakeCoralTest();
            case SCOREL4:
                return Level4Score();
            case SCOREL3:
                return LevelScore();
            case SCOREL2:
                return LevelScore();
            case SCOREL1:
                return LevelScore();
            case PROCESSOR:
                return algaeProcessorScore();
            default:
                return new InstantCommand(); // EQUIVALNT TO NULL, CHECK LATER TODO:
        }
    }

    public Command getLimelightAction() {
        switch (currentAction) {
            case INTAKE_ALGAE_HIGH:
                return TurnToAngle();
            case INTAKE_ALGAE_LOW:
                return TurnToAngle();
            case SCOREL4:
                return left ? DriveToTargetOffset4() : DriveToTargetOffset3();
            case SCOREL3:
                return left ? DriveToTargetOffset4() : DriveToTargetOffset3();
            case SCOREL2:
                return left ? DriveToTargetOffset4() : DriveToTargetOffset3();
            case SCOREL1:
                return left ? DriveToTargetOffset4() : DriveToTargetOffset3();
            default:
                return new InstantCommand(); // EQUIVALNT TO NULL, CHECK LATER TODO:
        }
    }

    public void setDesiredAction(desiredAction currAction) {
        currentAction = currAction;
    }
} 
