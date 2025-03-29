package frc.robot.commands;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ScoreSecondL4 extends SequentialCommandGroup {
    ManipulatorSubsystem manipulator;
    ElevatorSubsystem elevator;

    public ScoreSecondL4(ManipulatorSubsystem inManipulator, ElevatorSubsystem inElevatorSubsystem) {
        manipulator = inManipulator;
        elevator = inElevatorSubsystem;
        addCommands(
            Commands.print("test"),
            Commands.runOnce(() -> elevator.setTargetPosition(elevator.kL4SETPOINT)),
            new ParallelRaceGroup(
                Commands.run(() -> elevator.usePIDOutput(), elevator).withTimeout(2),
                new SequentialCommandGroup(
                    Commands.waitSeconds(1.3),
                    new ShootCoral(inManipulator),
                    Commands.print("shot")
                )
            )
        );
        addRequirements(elevator);
    }

}