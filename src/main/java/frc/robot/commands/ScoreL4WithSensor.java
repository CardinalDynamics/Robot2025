package frc.robot.commands;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
            Commands.print("test"),
            Commands.runOnce(() -> elevator.setTargetPosition(elevator.kL4SETPOINT)),
            new ParallelRaceGroup(
                Commands.run(() -> elevator.usePIDOutput(), elevator).withTimeout(1.5),
                new SequentialCommandGroup(
                    Commands.waitSeconds(1.45),
                    new ShootCoral(inManipulator),
                    Commands.print("shot")
                )
            ),
            Commands.print("set changed"),
            Commands.runOnce(() -> elevator.setTargetPosition(5), elevator),
            Commands.print("elevator"),
            Commands.run(() -> elevator.usePIDOutput(), elevator).withTimeout(1.5).finallyDo(() -> System.out.println("enddd")),
            Commands.print("end")
        );
        addRequirements(elevator, manipulator);
    }

}
 