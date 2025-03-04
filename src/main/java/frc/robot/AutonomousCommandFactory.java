// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.waitUntilPosition;
import frc.robot.commands.BasicCommands.IntakeCommand;
import frc.robot.commands.BasicCommands.RequestStateChange;
import frc.robot.commands.BasicCommands.WristCommand;
import frc.robot.commands.ComplexCommands.returnToIdle;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.IntakeWristRev;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;

/** Add your docs here. */
public class AutonomousCommandFactory extends CommandFactory{
    private boolean leftLimelight;
    private Elevator elevator;
    private IntakeRollers intakeRollers;
    private IntakeWristRev intakeWrist;
    private LimelightSubsystem Limelight4;
    private LimelightSubsystem Limelight3;
    private LimelightSubsystem Limelight2;
    private CommandSwerveDrivetrain swerve;
    private StateManager stateManager;

    public AutonomousCommandFactory( Elevator elevator, IntakeRollers intakeRollers, IntakeWristRev intakeWrist,
            LimelightSubsystem Limelight4, LimelightSubsystem Limelight3, LimelightSubsystem Limelight2,
            CommandXboxController driver, CommandSwerveDrivetrain swerve, StateManager stateManager) {
        super(elevator, intakeRollers, intakeWrist, Limelight4, Limelight3, Limelight2, driver, swerve, stateManager);
        this.elevator = elevator;
        this.intakeRollers = intakeRollers;
        this.intakeWrist = intakeWrist;
        this.Limelight4 = Limelight4;
        this.Limelight3 = Limelight3;
        this.Limelight2 = Limelight2;
        this.swerve = swerve;
        this.stateManager = stateManager;
    }

    // public Command autoLevelFour() {
    //     return new SequentialCommandGroup(
    //         new RequestStateChange(States.L4, stateManager), 
    //         new waitUntilPosition(stateManager),
    //         new WristCommand(intakeWrist, 40),
    //             new ParallelDeadlineGroup(
    //                 new WaitCommand(0.5),
    //                 new IntakeCommand(intakeRollers, .2, false),
    //                 new WristCommand(intakeWrist, 40)),
    //             new returnToIdle(stateManager)
    //     );
    // }

    // public Command autoLevelScore(int index) {
    //     return new SequentialCommandGroup(
    //             new RequestStateChange(index == 1 ? States.L1 : index == 2 ? States.L2 : States.L3, stateManager),
    //             new waitUntilPosition(stateManager),
    //             new WristCommand(intakeWrist, 40),
    //             new ParallelDeadlineGroup(
    //                 new WaitCommand(.5),
    //                 new IntakeCommand(intakeRollers, .2, false),
    //                 new WristCommand(intakeWrist, 40)),
    //             new RequestStateChange(States.IDLE, stateManager));
    // }

    public Command autoLevel4Position() {
        return new RequestStateChange(States.L4, stateManager);
    }
    
    public Command autoLevel4Score() {
        return new SequentialCommandGroup(
                new WristCommand(intakeWrist, 40),
                new ParallelDeadlineGroup(
                    new WaitCommand(0.5),
                    new IntakeCommand(intakeRollers, .2, false),
                    new WristCommand(intakeWrist, 40)),
                new returnToIdle(stateManager)
               );
    }

    public Command autoLevelScore() {
        return new SequentialCommandGroup(
                new WristCommand(intakeWrist, 40),
                new ParallelDeadlineGroup(
                    new WaitCommand(.5),
                    new IntakeCommand(intakeRollers, .2, false),
                    new WristCommand(intakeWrist, 40)),
                new RequestStateChange(States.IDLE, stateManager));
    }

    public Command autoLevelPosition(int index) {
        return new RequestStateChange(
            index == 1 ? States.L1 : index == 2 ? States.L2 : States.L3, stateManager);
    }

    public Command autoAlgaeLow() {
        return new SequentialCommandGroup(
            new RequestStateChange(States.INTAKE_ALGAE_LOW, stateManager),
            new waitUntilPosition(stateManager),
            new RequestStateChange(States.IDLE_ALGAE, stateManager));
    }

    public Command autoAlgaeHigh() {
        return new SequentialCommandGroup(
            new RequestStateChange(States.INTAKE_ALGAE_LOW, stateManager),
            new waitUntilPosition(stateManager),
            new RequestStateChange(States.IDLE_ALGAE, stateManager));
    }

    public void configAutonomousCommands() {
        NamedCommands.registerCommand("LevelOnePosition", autoLevelPosition(1));
        NamedCommands.registerCommand("LevelTwoPosition", autoLevelPosition(2));
        NamedCommands.registerCommand("LevelThreePosition", autoLevelPosition(3));
        NamedCommands.registerCommand("LevelFourPosition", autoLevel4Position());
        NamedCommands.registerCommand("LevelScore", autoLevelScore());
        NamedCommands.registerCommand("LevelFourScore", autoLevel4Score());
        NamedCommands.registerCommand("AlgaeLow", autoAlgaeLow());
        NamedCommands.registerCommand("AlgaeHigh", autoAlgaeHigh());
        NamedCommands.registerCommand("TurnToAngle", TurnToAngle());
        NamedCommands.registerCommand("DriveToOffsetLeft", DriveToTargetOffset4());
        NamedCommands.registerCommand("DriveToOffsetRight", DriveToTargetOffset3());
        NamedCommands.registerCommand("LockOnAprilTag", LockOnAprilTag());
        
    }

}
