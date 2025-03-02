// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;
import swervelib.parser.SwerveParser;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.IntakeWristRev;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.commands.BasicCommands.ElevatorCommand;
import frc.robot.commands.BasicCommands.IntakeCommand;
import frc.robot.commands.BasicCommands.RequestStateChange;
import frc.robot.commands.BasicCommands.WristCommand;
import frc.robot.commands.ComplexCommands.returnToIdle;
import frc.robot.commands.DefaultCommands.DefaultClimberCommand;
import frc.robot.commands.DefaultCommands.DefaultElevatorCommand;
import frc.robot.commands.DefaultCommands.DefaultRollerCommand;
import frc.robot.commands.DefaultCommands.DefaultWristCommand;
import frc.robot.commands.DriveCommands.DriveToLimeLightVisionOffset;
import frc.robot.commands.DriveCommands.DriveToTargetOffset;
import frc.robot.commands.DriveCommands.LockOnAprilTag;
import frc.robot.commands.DriveCommands.TurnToAngle;
import frc.robot.generated.TunerConstants;
import frc.robot.libraries.external.drivers.Limelight;
import frc.robot.state.IDLE;
import frc.robot.state.INTERPOLATED_STATE;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.06).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric().withDeadband(MaxSpeed * 0.025).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController pilot = new CommandXboxController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);

    //public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final StateManager stateManager = new StateManager();
    private final Climber climber = new Climber();
    private final Elevator elevator = new Elevator(stateManager);
    private final IntakeRollers rollers = new IntakeRollers();
    private final IntakeWristRev wrist = new IntakeWristRev(stateManager);

    

    
    // private final LimelightSubsystem limelight3 = new LimelightSubsystem( drivetrain,"limelight-three",0,0,0);
    // private final LimelightSubsystem limelight2 = new LimelightSubsystem(drivetrain,"limelight-two",0,0,0);
    // private final LimelightSubsystem limelight4 = new LimelightSubsystem(drivetrain, "limelight-four",0,0,0);

    //private final CommandFactory commandFactory = new CommandFactory( elevator, rollers, wrist, limelight4, limelight3, limelight2, copilot, drivetrain, stateManager);

    //private final cheaterDPAD dpad = new cheaterDPAD(commandFactory, stateManager);

    public RobotContainer() {
        stateManager.requestNewState(States.IDLE);
        //drivetrain.seedFieldCentric();
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-pilot.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-pilot.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-pilot.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        //  joystick.x().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //  joystick.y().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //  joystick.a().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //  joystick.b().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        elevator.setDefaultCommand(new DefaultElevatorCommand(elevator, stateManager));
        wrist.setDefaultCommand(new DefaultWristCommand(wrist, stateManager));
        rollers.setDefaultCommand(new DefaultRollerCommand(rollers, stateManager));

        climber.setDefaultCommand(new DefaultClimberCommand(climber, stateManager, copilot)); 

        //pilot.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        // pilot.b().onTrue(commandFactory.algaeProcessorPosition());
        // pilot.b().onFalse(commandFactory.algaeProcessorScore());
        // pilot.x().onTrue(commandFactory.algaeLowPosition());
        // pilot.x().onFalse(commandFactory.algaeIdle());
        // pilot.a().onTrue(commandFactory.algaeHighPosition());
        // pilot.a().onFalse(commandFactory.algaeIdle());
        // pilot.start().onTrue(new RequestStateChange(States.IDLE, stateManager));

        // pilot.pov(0).onTrue(commandFactory.Level4Position());
        // pilot.pov(0).onFalse(commandFactory.Level4Score());
        // pilot.pov(90).onTrue(commandFactory.LevelPosition(3));
        // pilot.pov(90).onFalse(commandFactory.LevelScore());
        // pilot.pov(180).onTrue(commandFactory.LevelPosition(2));
        // pilot.pov(180).onFalse(commandFactory.LevelScore());
        // pilot.pov(270).onTrue(commandFactory.LevelPosition(1));
        // pilot.pov(270).onFalse(commandFactory.LevelScore());

    //     pilot.leftTrigger().onTrue(commandFactory.LevelPosition(1));
    //     pilot.leftTrigger().onFalse(commandFactory.LevelScore());
    //     pilot.leftBumper().onTrue(commandFactory.LevelPosition(2));
    //     pilot.leftBumper().onFalse(commandFactory.LevelScore());
    //     pilot.rightTrigger().onTrue(commandFactory.Level4Position());
    //     pilot.rightTrigger().onFalse(commandFactory.Level4Score());
    //     pilot.rightBumper().onTrue(commandFactory.LevelPosition(3));
    //     pilot.rightBumper().onFalse(commandFactory.LevelScore());

    //     pilot.b().whileTrue(new InstantCommand( ()-> drivetrain.applyRequest(() ->
    //     robotCentricDrive.withVelocityX(-pilot.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
    //         .withVelocityY(-pilot.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    //         .withRotationalRate(-pilot.getRightX() * MaxAngularRate)) // Drive counterclockwise with negative X (left)
    // ) );

    //     pilot.x().whileTrue(new LockOnAprilTag(drivetrain, limelight3, 0, pilot, false));
    //     pilot.a().whileTrue(new TurnToAngle(drivetrain, limelight3));
    //     //CommandSwerveDrivetrain drivetrain, LimelightSubsystem Limelight, double offset, int pipeline, double targetx, double targety
    //     pilot.pov(0).whileTrue(new DriveToTargetOffset(drivetrain, limelight3, 0, 0, -22, 13));
    //  //   pilot.b().whileTrue(dpad.reefLimelight(drivetrain, limelight4));
    //     pilot.pov(90).whileTrue(new DriveToTargetOffset(drivetrain, limelight4, 0, 0, -22, 13));
    //     pilot.pov(180).whileTrue(new TurnToAngle(drivetrain, limelight4));


    //     copilot.pov(0).onTrue(commandFactory.algaeProcessorPosition());
    //     copilot.pov(0).onFalse(commandFactory.algaeProcessorScore());
    //     copilot.b().onTrue(commandFactory.algaeLowPosition());
    //     copilot.b().onFalse(commandFactory.algaeIdle());
    //     copilot.y().onTrue(commandFactory.algaeHighPosition());
    //     copilot.y().onFalse(commandFactory.algaeIdle());
        //copilot.rightTrigger().onTrue(commandFactory.IntakeCoralTest());

        // copilot.a().whileTrue(new InstantCommand(() -> climber.climberOn(.9)));
        // copilot.a().onFalse(new InstantCommand(() -> climber.climberOn(0)));
        // copilot.x().whileTrue(new InstantCommand(() -> climber.climberOn(-.9)));
        // copilot.x().onFalse(new InstantCommand(() -> climber.climberOn(0)));

        // climber.climberOn(Math.abs(copilot.getLeftX())<0.07 ? 0 : copilot.getLeftX());
        // SmartDashboard.putNumber("climberspeed", Math.abs(copilot.getLeftX())<0.07 ? 0 : copilot.getLeftX()/3);

        // copilot.back().onTrue(new InstantCommand(() -> dpad.rightLimeLight()));
        // copilot.start().onTrue(new InstantCommand(() -> dpad.leftLimeLight()));
        

    //pilot.rightTrigger().onTrue(commandFactory.IntakeCoralTest());

        // pilot.pov(0).onTrue(dpad.fetchUpPress());
        // pilot.pov(0).onTrue(dpad.fetchUpPress());
        // pilot.pov(90).onTrue(dpad.fetchRightPress());
        // pilot.pov(90).onTrue(dpad.fetchRightPress());
        // pilot.pov(180).onTrue(dpad.fetchDownPress());
        // pilot.pov(180).onTrue(dpad.fetchDownPress());
        // pilot.pov(270).onTrue(dpad.fetchLeftPress());
        // pilot.pov(270).onTrue(dpad.fetchLeftPress());

        // pilot.back().onTrue(dpad.algaeMode());
        // pilot.start().onTrue(dpad.coralMode());

         

        //OPTIONAL STUFF TO TEST LATER
        // pilot.rightBumper().whileTrue(new LockOnAprilTag(drivetrain, limelight3, 0, pilot, false));
        // pilot.leftBumper().whileTrue(new TurnToAngle(drivetrain, 90.0));
        // //test if this works or not
        // pilot.leftTrigger(.5).whileTrue(new DriveToTargetOffset(drivetrain, limelight3, 0, 0, -16.93,  11.7));
        // pilot.rightTrigger().whileTrue(new DriveToLimeLightVisionOffset(drivetrain, limelight3, 0, false));


    copilot.rightTrigger(.7).onTrue(new RequestStateChange(States.INTAKE_CORAL, stateManager));
    
    copilot.rightTrigger(.7).onFalse(new SequentialCommandGroup(
        new ParallelCommandGroup(new WaitCommand(1.5), new ElevatorCommand(elevator, 8.8)),
        new ParallelRaceGroup(new WaitCommand(0.25), new ElevatorCommand(elevator, 9.375)),
        new ParallelDeadlineGroup(new WaitCommand(1), new WristCommand(wrist, 60)), 
        new RequestStateChange(States.IDLE, stateManager)
    ));
  
        //drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
