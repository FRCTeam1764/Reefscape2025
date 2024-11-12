package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.*;
import frc.robot.commands.DriveCommands.DoA180;
import frc.robot.commands.DriveCommands.LockOnAprilTag;
import frc.robot.commands.DriveCommands.TeleopDrive;
import frc.robot.constants.CommandConstants;
import frc.robot.constants.SwerveConstantsYAGSL;
import frc.robot.subsystems.*;
import frc.robot.libraries.external.control.Path;
import frc.robot.libraries.external.control.Trajectory;
import frc.robot.libraries.external.robot.input.JoystickAxis;
import frc.robot.state.RobotState;

import java.io.File;

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
    private final JoystickAxis LeftTrigger = new JoystickAxis(driver, XboxController.Axis.kLeftTrigger.value);
    private final JoystickButton FLIPURSELF = new JoystickButton(driver, XboxController.Button.kX.value);

    private final JoystickButton SpeakerLimelight = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton RingLimelight = new JoystickButton(driver, XboxController.Button.kB.value);

    /* CoPilot Buttons */

    /* Subsystems */

    public RobotState robotState = new RobotState(driver);

    private  SendableChooser<Command> autoChooser;
    //private final Music THEMUSIC = new Music();
    private final Superstructure superstructure = new Superstructure();
    
    private final Blinkin blinky = new Blinkin();

    private final SwerveSubsystem s_Swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/falcon"));
    // Limelights
    // limelight 3 is front intake
    // limelight 2 is back shooter
    // 4,7,15,16,14,12,11,13,6,5 - april id tags that limelight 2 should see
    private LimelightSubsystem limelight3 = new LimelightSubsystem("limelight-three", 1,s_Swerve);
    private LimelightSubsystem limelight2 = new LimelightSubsystem("limelight-two",0,s_Swerve);

    private Trajectory[] trajectories;

    public RobotContainer() {

        // teleop drive for yagsl
    limelight3.setPipeline(1);

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

        // autoChooser =  AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(autoChooser);
        
    }

    private void configurePilotButtonBindings() {
        //y
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        //a
        FLIPURSELF.whileTrue(new DoA180(s_Swerve, driver, true));
    }

    private void configureCoPilotButtonBindings() {
        
    }

    public void configAutoCommands() {
        
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

    public Superstructure getSuperstructure() {
        return superstructure;
    }

    public Joystick getsecondaryController() {
        return secondaryController;
    }

    public Joystick getPrimaryController() {
        return driver;
    }

    public Trajectory[] getTrajectories() {
        return trajectories;
    }
}
