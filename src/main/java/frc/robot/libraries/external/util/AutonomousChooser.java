// package frc.robot.libraries.external.util;

// import java.util.ArrayList;
// import java.util.HashMap;
// import java.util.List;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.FollowPathWithEvents;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.util.*;
// import edu.wpi.first.hal.simulation.RoboRioDataJNI;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.PrintCommand;
// import frc.robot.commands.*;
// import frc.robot.libraries.external.control.Trajectory;

// import frc.robot.RobotContainer;
// //TO DO make this work with pathplaner instead of whatever rn 
// public class AutonomousChooser {
//     private final Trajectory[] trajectories;
//     private final  HashMap<String, Command> eventMap;
//     private final AutoBuilder autoBuilder = new AutoBuilder();
    
//     private final SendableChooser<Command> autonomousModeChooser = AutoBuilder.buildAutoChooser();

//     private enum EX {
//         DEFAULT,
//         AUTOBALANCELEFT,
//     }

//     public AutonomousChooser(Trajectory[] trajectories,RobotContainer robotContainer) {
//             this.trajectories = trajectories;
//             SmartDashboard.putData("Auto Chooser", autonomousModeChooser);
    
//             // autonomousModeChooser.setDefaultOption("Default", AutonomousMode.DEFAULT);
//             // autonomousModeChooser.addOption("AutoBalanceLeftSide", AutonomousMode.AUTOBALANCELEFT);
    
    
//             eventMap = new HashMap<>();
//           //  eventMap.put("PlacePieceCone", new AutoIntakeCommand(robotContainer.getIntake(),false,"Cone"));
//           AutoBuilder.configureHolonomic(
//             robotContainer.getDrivetrainSubsystem()::getPose, // Robot pose supplier
//              robotContainer.getDrivetrainSubsystem()::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
//              robotContainer.getDrivetrainSubsystem()::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
//              robotContainer.getDrivetrainSubsystem()::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
//             new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
//                     new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
//                     new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
//                     4.5, // Max module speed, in m/s
//                     0.4, // Drive base radius in meters. Distance from robot center to furthest module.
//                     new ReplanningConfig() // Default path replanning config. See the API for the options here
//             ),
//             () -> {
//                 // Boolean supplier that controls when the path will be mirrored for the red alliance
//                 // This will flip the path being followed to the red side of the field.
//                 // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

//                 var alliance = DriverStation.getAlliance();
//                 if (alliance.isPresent()) {
//                     return alliance.get() == DriverStation.Alliance.Red;
//                 }
//                 return false;
//             },
//             robotContainer.getDrivetrainSubsystem() // Reference to this subsystem to set requirements
//     );
// //  );


// //            autoBuilder.configureHolonomic(
// //     robotContainer.getDrivetrainSubsystem()::getPose, // Pose2d supplier
// //     robotContainer.getDrivetrainSubsystem()::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
// //     robotContainer.getDrivetrainSubsystem().kinematics, // SwerveDriveKinematics
// //     robotContainer.getDrivetrainSubsystem()::setModuleStates, // Module states consumer used to output to the drive subsystem
// //     new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
// //     new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
// //     eventMap,
// //     true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
// //     robotContainer.getDrivetrainSubsystem() // The drive subsystem. Used to properly set the requirements of path following commands
// // );

//     }

//     public SendableChooser<Command> getAutonomousModeChooser() {
//         return autonomousModeChooser;
//     }

//     private Command getDefaultAutoCommand(RobotContainer robotContainer) {
// return new InstantCommand();
//     }

//     private Command getAUTOBALANCELEFTAutoCommand(RobotContainer robotContainer) {
//         List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("ScoreSpeaker");
// ;
//         Command fullAuto = null;//autoBuilder.pathfindThenFollowPath(pathGroup);


//         return fullAuto; //FollowTrajectoryCommand(robotContainer.getDrivetrainSubsystem(), trajectories[0]);
//     }


//     public Command getCommand(RobotContainer container) {




//         return autonomousModeChooser.getSelected();
//     //     switch (autonomousModeChooser.getSelected()) {
//     //         case DEFAULT:
//     //             return getDefaultAutoCommand(container);
//     //         case AUTOBALANCELEFT:
//     //             return getAUTOBALANCELEFTAutoCommand(container);
//     //     }

//     //     return getDefaultAutoCommand(container);
//     // }
//     }
// }