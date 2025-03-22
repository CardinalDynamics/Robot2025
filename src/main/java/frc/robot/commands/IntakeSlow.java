package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

public class IntakeSlow extends Command {
    ManipulatorSubsystem manipulator;
    FunnelSubsystem funnel;
    
    public IntakeSlow(ManipulatorSubsystem manipulatorIn, FunnelSubsystem funnelIn) {
        manipulator = manipulatorIn;
        funnel = funnelIn;
        addRequirements(manipulatorIn, funnelIn);
    }

    public void initialize() {
        funnel.setFunnelVoltage(-4);
        manipulator.setManipulatorVoltage(1.0);
    }

    public void end() {
        funnel.setFunnelVoltage(0);
        manipulator.setManipulatorVoltage(0);
    }

    public boolean isFinished() {
        return manipulator.sensorMeasurement() < 50;
    }
}
