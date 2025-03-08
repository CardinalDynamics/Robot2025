package frc.robot.commands;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ScoreL4WithSensor extends SequentialCommandGroup {
    ManipulatorSubsystem manipulator;
    ElevatorSubsystem elevator;

    public ScoreL4WithSensor(ManipulatorSubsystem inManipulator, ElevatorSubsystem inElevatorSubsystem) {
        manipulator = inManipulator;
        elevator = inElevatorSubsystem;
        addCommands(
            Commands.runOnce(() -> elevator.setTargetPosition(elevator.kL4SETPOINT)),
            new ParallelCommandGroup(
                Commands.run(() -> elevator.usePIDOutput(), elevator).until(() -> !manipulator.hasCoral()),
                new SequentialCommandGroup(
                    Commands.waitSeconds(1.4),
                    new ShootCoral(inManipulator)
                )
            ),
            Commands.runOnce(() -> elevator.setTargetPosition(5)),
            Commands.run(() -> elevator.usePIDOutput(), elevator).withTimeout(1.3)
        );
        addRequirements(elevator, manipulator);
    }
}
 