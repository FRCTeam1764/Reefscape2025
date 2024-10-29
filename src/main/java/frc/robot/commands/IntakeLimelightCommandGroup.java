// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import frc.robot.subsystems.LimelightSubsystem;
// import frc.robot.subsystems.intakeSubsystem;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class IntakeLimelightCommandGroup extends ParallelCommandGroup {
//   /** Creates a new IntakeLimelightCommandGroup. */
//   public IntakeLimelightCommandGroup(
//     LimelightSubsystem limelight,
//     int pipeline,
//     intakeSubsystem intake,
//     double intakeSpeed,
//     boolean intakeClose,
//     int heightLevel
//   ) {

//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(
//       new LimelightCommand(limelight, pipelineChange(intakeClose, intake)),
//       new intakeCommand(intake, intakeSpeed, intakeClose, heightLevel)
//     );
//   }
//   private int pipelineChange(boolean intakeClose, intakeSubsystem intake){
//     if(!intakeClose){
//       return 2;
//     }
//     else if(intake.getColor1() && intake.getColor2()){
//       return 1;
//     }
//     return 0;
//   }
// }
