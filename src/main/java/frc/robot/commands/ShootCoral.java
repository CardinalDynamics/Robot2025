package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ShootCoral extends Command {
    ManipulatorSubsystem manipulator;
    
    public ShootCoral(ManipulatorSubsystem manipulatorIn) {
        manipulator = manipulatorIn;
        addRequirements(manipulatorIn);
    }

    public void initialize() {
        manipulator.setManipulatorVoltage(4);
    }

    public void end() {
        manipulator.setManipulatorVoltage(0);
    }

    public boolean isFinished() {
        return manipulator.sensorMeasurement() > 120;
    }
}