package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

public class IntakeAuto extends Command {
    ManipulatorSubsystem manipulator;
    FunnelSubsystem funnel;
    LEDSubsystem leds;
    boolean passed = false;
    
    public IntakeAuto(ManipulatorSubsystem manipulatorIn, FunnelSubsystem funnelIn) {
        manipulator = manipulatorIn;
        funnel = funnelIn;
        addRequirements(manipulatorIn, funnelIn);
    }

    public void initialize() {
        passed = false;
        funnel.setFunnelVoltage(-8);
        manipulator.setManipulatorVoltage(4);
    }

    public void execute() {
        if (funnel.elevatorBlocked()) {
            passed = true;
            funnel.setFunnelVoltage(-8);
        }
        if (passed && !funnel.elevatorBlocked()) {
            funnel.setFunnelVoltage(0);
            manipulator.setManipulatorVoltage(2);
        }
    }

    public void end() {
        funnel.setFunnelVoltage(0);
        manipulator.setManipulatorVoltage(0);
        passed = false;
    }

    public boolean isFinished() {
        return manipulator.hasCoral();
    }
}
