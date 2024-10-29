package frc.robot.libraries.external.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.*;
import frc.robot.libraries.external.control.Trajectory;

import frc.robot.RobotContainer;
//TO DO make this work with pathplaner instead of whatever rn 
public class AutonomousChooser {
    private final Trajectory[] trajectories;
    private final  HashMap<String, Command> eventMap;
    private final SwerveAutoBuilder autoBuilder;
    
    private SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    private enum AutonomousMode {
        DEFAULT,
        AUTOBALANCELEFT,
    }

    public AutonomousChooser(Trajectory[] trajectories,RobotContainer robotContainer) {
        this.trajectories = trajectories;

        autonomousModeChooser.setDefaultOption("Default", AutonomousMode.DEFAULT);
        autonomousModeChooser.addOption("AutoBalanceLeftSide", AutonomousMode.AUTOBALANCELEFT);


        eventMap = new HashMap<>();
      //  eventMap.put("PlacePieceCone", new AutoIntakeCommand(robotContainer.getIntake(),false,"Cone"));


        autoBuilder = new SwerveAutoBuilder(
    robotContainer.getDrivetrainSubsystem()::getPose, // Pose2d supplier
    robotContainer.getDrivetrainSubsystem()::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
    robotContainer.getDrivetrainSubsystem().kinematics, // SwerveDriveKinematics
    new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    robotContainer.getDrivetrainSubsystem()::setModuleStates, // Module states consumer used to output to the drive subsystem
    eventMap,
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    robotContainer.getDrivetrainSubsystem() // The drive subsystem. Used to properly set the requirements of path following commands
);

    }

    public SendableChooser<AutonomousMode> getAutonomousModeChooser() {
        return autonomousModeChooser;
    }

    private Command getDefaultAutoCommand(RobotContainer robotContainer) {
return new InstantCommand();
    }

    private Command getAUTOBALANCELEFTAutoCommand(RobotContainer robotContainer) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("OnePieceLeft", new PathConstraints(4, 3));
        Command fullAuto = autoBuilder.fullAuto(pathGroup);


        return fullAuto; //FollowTrajectoryCommand(robotContainer.getDrivetrainSubsystem(), trajectories[0]);
    }


    public Command getCommand(RobotContainer container) {
        switch (autonomousModeChooser.getSelected()) {
            case DEFAULT:
                return getDefaultAutoCommand(container);
            case AUTOBALANCELEFT:
                return getAUTOBALANCELEFTAutoCommand(container);
        }

        return getDefaultAutoCommand(container);
    }
}