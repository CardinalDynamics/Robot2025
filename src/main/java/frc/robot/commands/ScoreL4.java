// package frc.robot.commands;

// import au.grapplerobotics.LaserCan;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.ManipulatorSubsystem;

// public class ScoreL4 extends SequentialCommandGroup {
//     ManipulatorSubsystem manipulator;
//     ElevatorSubsystem elevator;

//     public ScoreL4(ManipulatorSubsystem inManipulator, ElevatorSubsystem inElevatorSubsystem) {
//         manipulator = inManipulator;
//         elevator = inElevatorSubsystem;
//         addCommands(
//             Commands.runOnce(() -> elevator.setTargetPosition(elevator.kL4SETPOINT)),
//             new ParallelCommandGroup(
//                 Commands.run(() -> elevator.usePIDOutput(), elevator).withTimeout(1),
//                 new SequentialCommandGroup(
//                     Commands.waitUntil(() -> elevator.atGoal()),
//                     Commands.run(() -> manipulator.setManipulatorVoltage(8)),
//                     Commands.waitSeconds(.3),
//                     Commands.run(() -> manipulator.setManipulatorVoltage(0))
//                 )
//             ),
//             Commands.runOnce(() -> elevator.setTargetPosition(5)),
//             Commands.run(() -> elevator.usePIDOutput(), elevator).withTimeout(.5)
//         );
//     }
// }
 