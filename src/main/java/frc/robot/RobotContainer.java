package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.CommandFactory.desiredAction;
import frc.robot.commands.*;
import frc.robot.commands.BasicCommands.RequestStateChange;
import frc.robot.commands.DefaultCommands.DefaultElevatorCommand;
import frc.robot.commands.DefaultCommands.DefaultWristCommand;
import frc.robot.commands.DriveCommands.LockOnAprilTag;
import frc.robot.commands.DriveCommands.TeleopDrive;
import frc.robot.constants.CommandConstants;
import frc.robot.constants.SwerveConstantsYAGSL;
import frc.robot.subsystems.*;
import frc.robot.subsystems.StateManager.States;
import frc.robot.libraries.external.robot.input.JoystickAxis;

import java.io.File;

import javax.sql.StatementEvent;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RobotContainer {
    
    private final Joystick driver = new Joystick(0);
    private final Joystick secondaryController = new Joystick(1); 

    /* Drive Controls */

    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */

    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton actionButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton FLIPURSELF = new JoystickButton(driver, XboxController.Button.kX.value);


    /* CoPilot Buttons */
    private final JoystickButton test1 = new JoystickButton(secondaryController, XboxController.Button.kA.value);
    /* Subsystems */

    private final StateManager stateManager = new StateManager();
    
    private final IntakeRollers intakeRollers = new IntakeRollers();
    private final IntakeWrist intakeWrist = new IntakeWrist(stateManager);
    private final Elevator elevator = new Elevator(stateManager);
    private final Climber climber = new Climber();
    
    private final Blinkin blinky = new Blinkin();


    private final SwerveSubsystem s_Swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/falcon"));
//TODO: FIND LIMELIGHT NAMES

    //left
    private final LimelightSubsystem limelight4 = new LimelightSubsystem(s_Swerve, "LIMELIGHT4", 0, 0,0);
    //right
    private final LimelightSubsystem limelight3 = new LimelightSubsystem(s_Swerve, "LIMELIGHT3", 0, 0,0);
    //back
    private final LimelightSubsystem limelight2 = new LimelightSubsystem(s_Swerve, "LIMELIGHT2", 0, 0,0);

    

    
    private final CommandFactory commandFactory = new CommandFactory(climber, elevator,intakeRollers,intakeWrist,limelight4,limelight3,limelight2,driver,s_Swerve,stateManager);
    
    
    private  SendableChooser<Command> autoChooser;


    public RobotContainer() {

        // teleop drive for yagsl 
    
    
        s_Swerve.setDefaultCommand(
                new TeleopDrive(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> !robotCentric.getAsBoolean()));

        configAutoCommands();
        configurePilotButtonBindings();
        configureCoPilotButtonBindings();

        SmartDashboard.putData(autoChooser);
        
        elevator.setDefaultCommand(new DefaultElevatorCommand(elevator,stateManager));
        intakeWrist.setDefaultCommand(new DefaultWristCommand(intakeWrist, stateManager));
        
    }

    private void configurePilotButtonBindings() {


        //y
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        
        actionButton.onTrue(new InstantCommand(() -> commandFactory.getDesiredAction()));
    }

    private void configureCoPilotButtonBindings() {
        
        test1.onTrue(new InstantCommand(() -> commandFactory.setDesiredAction(desiredAction.INTAKE)));
    }

    public void configAutoCommands() {
        NamedCommands.registerCommand(null, getAutonomousCommand());
    }


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public SwerveSubsystem getDrivetrainSubsystem(){
     return s_Swerve;
     }
    public double getPercentFromBattery(double speed){
        return speed * 12 / RobotController.getBatteryVoltage();
    }
}
