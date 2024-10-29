// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.AutoElevatorCommand;
// import frc.robot.commands.AutoIntakeCommand;
// import frc.robot.commands.AutoPivotyCommand;
// import frc.robot.commands.BlinkinCommand;
// import frc.robot.commands.ElevatorPivotyCommandGroup;
// import frc.robot.state.ElevatorState;
// import frc.robot.state.PivotyState;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.PivotySubsystem;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class OnePiece extends SequentialCommandGroup {
//   /** Creates a new OnePiece. */
//   public OnePiece(PivotySubsystem pivoty, PivotyState pivotyState, Elevator elevator, ElevatorState elevatorState, Intake intake) {
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//      addCommands(
//       new AutoPivotyCommand(pivoty,70000,pivotyState),
//       new AutoElevatorCommand(elevator, elevatorState, -115000),

//       new AutoIntakeCommand(intake, -0.5),

//       new AutoElevatorCommand(elevator, elevatorState, 0),
//       new AutoPivotyCommand(pivoty,0,pivotyState)

//      );
//   }
// }
