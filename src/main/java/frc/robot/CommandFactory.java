// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
import frc.robot.commands.DriveCommands.DriveForward;
import frc.robot.commands.DriveCommands.DriveToTarget;
import frc.robot.commands.DriveCommands.DriveToTargetOffset;
import frc.robot.commands.DriveCommands.DriveToTargetOffsetLL3;
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

    public Command LevelPosition(int index) {
        return new RequestStateChange(
            index == 1 ? States.L1 : index == 2 ? States.L2 : States.L3, stateManager);
    }

    public Command Level1Score(){
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(new WaitCommand(.5),new IntakeCommand(intakeRollers, .2, false)),
            new RequestStateChange(States.IDLE, stateManager)
            );
    }

    public Command LevelScore() {
        return new ConditionalCommand(new SequentialCommandGroup(
                new WristCommand(intakeWrist, 45),
                new ParallelDeadlineGroup(
                    new WaitCommand(.5),
                    new IntakeCommand(intakeRollers, .2, false),
                    new WristCommand(intakeWrist, 45)),
                new ParallelRaceGroup(
                    new WaitCommand(.3),
                    new WristCommand(intakeWrist, 30)),
                new RequestStateChange(States.IDLE, stateManager)), 
                new RequestStateChange(States.IDLE, stateManager), 
                ()->stateManager.getWillScore());
    }

    public Command LevelScoreL2() { 
        return new ConditionalCommand(
            new SequentialCommandGroup(
                new WristCommand(intakeWrist, 40),
                new WaitCommand(.1),
                new ParallelDeadlineGroup(
                    new WaitCommand(.75),
                    new IntakeCommand(intakeRollers, .2, false),
                    new WristCommand(intakeWrist, 40)),
                new ParallelRaceGroup(
                    new WaitCommand(.3),
                    new WristCommand(intakeWrist, 30)),
            new RequestStateChange(States.IDLE, stateManager)), 
            new RequestStateChange(States.IDLE, stateManager), 
            () -> stateManager.getWillScore());
    }

    public Command Level4Position() {
        return new RequestStateChange(States.L4, stateManager);
    }
    
    public Command Level4Score() {
        return new ConditionalCommand(new SequentialCommandGroup(
            new ParallelRaceGroup(
                    new WristCommand(intakeWrist, 40),
                    new WaitCommand(.3)
            ),
                new ParallelDeadlineGroup(
                    new WaitCommand(0.5),
                    new IntakeCommand(intakeRollers, .2, false).asProxy(),
                    new WristCommand(intakeWrist, 50)),
                new ParallelRaceGroup(
                    new WaitCommand(.3),
                    new WristCommand(intakeWrist, 30)),
                new RequestStateChange(States.IDLE, stateManager)
               ), new RequestStateChange(States.IDLE, stateManager), ()->stateManager.getWillScore());
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
        return new RequestStateChange(States.IDLE_ALGAE, stateManager);
    }

    public Command algaeProcessorPosition() {
        return new RequestStateChange(States.PROCESSOR, stateManager);
    }

    public Command algaeBargePosition(){
        return new SequentialCommandGroup(
            new RequestStateChange(States.BARGE, stateManager),
            new waitUntilPositionIndex(stateManager, CommandConstants.ELEVATOR_KEY, 20),
            new ParallelDeadlineGroup(
                new WaitCommand(0.8), 
                new IntakeCommand(intakeRollers, 0.8, false)).asProxy(),
            new waitUntilPosition(stateManager),
            new returnToIdle(stateManager));
    }

    public Command algaeBargeBack() {
        return new returnToIdle(stateManager);
    }

    public Command algaeBargeScore(){
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
            new WaitCommand(1),
            new IntakeCommand(intakeRollers, .3, false).asProxy()),
        new returnToIdle(stateManager, States.IDLE));
    }
    

    public Command algaeProcessorScore() {
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                new waitUntilPosition(stateManager),
                new IntakeCommand(intakeRollers, .3, false).asProxy()),
            new RequestStateChange(States.IDLE, stateManager)
        );
    }

    public Command IntakeCoralTest() {
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                new WaitCommand(0.3),
                new ElevatorCommand(elevator, 8.25)),
            new WaitCommand(0.1),
            new ElevatorCommand(elevator, 13),
            new RequestStateChange(States.IDLE_CORAL, stateManager)
            );
    }

    public Command IntakeCoralPosition() {
        return new SequentialCommandGroup( 
            new ElevatorCommand(elevator, 10),
            new RequestStateChange(States.INTAKE_CORAL, stateManager));
    }

    public Command driveForward() {
        return new ParallelDeadlineGroup(new WaitCommand(0.15), new DriveForward(swerve));
    }

    //limalight
    public Command LockOnAprilTag() {
        return new LockOnAprilTag(swerve, Limelight3, 0, driver, false);
    }

    public Command TurnToAngle() {
        return new TurnToAngle(swerve, Limelight3);
    }

    public Command DriveToTargetOffsetRight() {
        return new DriveToTargetOffsetLL3(swerve, Limelight4, 0, 0, 17.3, 9.3);
    }

    public Command DriveToTargetOffsetLeft() {
        return new DriveToTargetOffset(swerve, Limelight3, 0, 0, -18.0, 14.8);
    }


    public Command interupted(boolean wasInteruppted) {
        if (wasInteruppted) {
            return new InstantCommand();
        }
        return new returnToIdle(stateManager);
    }



} 
