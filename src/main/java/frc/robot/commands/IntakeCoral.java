package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.ManipulatorSubsystem;

public class IntakeCoral extends Command {
    ManipulatorSubsystem manipulator;
    
    public IntakeCoral(ManipulatorSubsystem manipulatorIn) {
        manipulator = manipulatorIn;
        addRequirements(manipulatorIn);
    }

    public void initialize() {
        manipulator.setManipulatorVoltage(4);
    }

    public void end() {
        manipulator.setManipulatorVoltage(0);
        LimelightHelpers.setLEDMode_ForceBlink("");
    }

    public boolean isFinished() {
        return manipulator.sensorMeasurement() < 100;
    }
}
