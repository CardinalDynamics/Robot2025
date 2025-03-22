package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

public class IntakeCoral extends Command {
    ManipulatorSubsystem manipulator;
    FunnelSubsystem funnel;
    LEDSubsystem leds;
    boolean passed = false;
    
    public IntakeCoral(ManipulatorSubsystem manipulatorIn, FunnelSubsystem funnelIn, LEDSubsystem ledsIn) {
        manipulator = manipulatorIn;
        funnel = funnelIn;
        leds = ledsIn;
        addRequirements(manipulatorIn, funnelIn, ledsIn);
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
            leds.setBlinkyLights();
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
        leds.setCoralState();
    }

    public boolean isFinished() {
        return manipulator.hasCoral();
    }
}
