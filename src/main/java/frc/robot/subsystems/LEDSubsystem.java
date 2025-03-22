package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private static final int kPort = 9;
    private static final int kLength = 215;

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;

    // all hues at maximum saturation and half brightness
    public final LEDPattern m_rainbow = LEDPattern.rainbow(255, 255);

    // Our LED strip has a density of 120 LEDs per meter
    private static final Distance kLedSpacing = Meters.of(1 / 215.0);

    // Create a new pattern that scrolls the rainbow pattern across the LED strip,
    // moving at a speed
    // of 1 meter per second.
    private final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(10), kLedSpacing).blink(Seconds.of(.1));

    private final LEDPattern m_blinky = LEDPattern.solid(Color.kGreen).blink(Seconds.of(0.05));
    private final LEDPattern m_coralState = LEDPattern.solid(Color.kGreen);

    private LEDPattern m_default;

    private final Timer timer = new Timer();

    public LEDSubsystem() {
        if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) {
            m_default = LEDPattern.solid(Color.kRed).breathe(Seconds.of(2));
        } else {
            m_default = LEDPattern.solid(Color.kBlue).breathe(Seconds.of(2));
        }
        m_led = new AddressableLED(kPort);
        m_buffer = new AddressableLEDBuffer(kLength);
        m_led.setLength(kLength);
        m_led.start();

        timer.restart();
        setDefaultLights();
        // Set the default command to turn the strip off, otherwise the last colors
        // written by
        // the last command to run will continue to be displayed.
        // Note: Other default patterns could be used instead!
    }

    public void setEndgameLights() {
        m_scrollingRainbow.applyTo(m_buffer);
    }

    public void setBlinkyLights() {
        m_blinky.applyTo(m_buffer);
    }

    public void setCoralState() {
        m_coralState.applyTo(m_buffer);
    }

    public void setDefaultLights() {
        m_default.applyTo(m_buffer);

    }
    /**
     * Creates a command that runs a pattern on the entire LED strip.
     *
     * @param pattern the LED pattern to run
     */
    public Command runPattern() {
        return run(() -> m_scrollingRainbow.applyTo(m_buffer));
    }

    @Override
    public void periodic() {
        // Periodically send the latest LED color data to the LED strip for it to
        // display
        m_led.setData(m_buffer);
    }
}