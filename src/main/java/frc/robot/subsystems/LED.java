package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import com.ctre.phoenix6.signals.Led1OffColorValue;

public class LED extends SubsystemBase {
    private final AddressableLED leds;
    private final AddressableLEDBuffer ledBuffer;
    private final AddressableLEDBufferView leftLeds;
    private final AddressableLEDBufferView rightLeds;

    // Level Segments
    private LEDPattern leftL1Led;
    private LEDPattern rightL1Led;
    private LEDPattern leftL2Led;
    private LEDPattern rightL2Led;
    private LEDPattern leftL3Led;
    private LEDPattern rightL3Led;
    private LEDPattern leftL4Led;
    private LEDPattern rightL4Led;

    // Flashing and Solid Patterns
    private LEDPattern flashing;
    private LEDPattern solid;

    private Color colour = Color.kBlack;

    private RobotContainer robotContainer;

    public LED(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;

        leds = new AddressableLED(Constants.LEDConstants.channel);
        ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.length);
        leds.setLength(ledBuffer.getLength());

        leftLeds = new AddressableLEDBufferView(ledBuffer, 0, 75);
        rightLeds = new AddressableLEDBufferView(ledBuffer, 76, 150);

        // L1 LEDs
        leftL1Led = LEDPattern.steps(Map.of(0, colour, 0.25, Color.kBlack, 0.75, colour));
        rightL1Led = LEDPattern.steps(Map.of(0, Color.kBlack, 0.25, colour, 0.75, Color.kBlack));

        // L2 LEDs
        leftL2Led = LEDPattern.steps(Map.of(0, colour, 0.4, Color.kBlack, 0.6, colour));
        rightL2Led = LEDPattern.steps(Map.of(0, Color.kBlack, 0.4, colour, 0.6, Color.kBlack));

        // L3 LEDs
        leftL3Led = LEDPattern.steps(Map.of(0, colour, 0.6, Color.kBlack, 0.7, colour));
        rightL3Led = LEDPattern.steps(Map.of(0, Color.kBlack, 0.6, colour,
                0.7, Color.kBlack));

        // L4 LEDs
        leftL4Led = LEDPattern.steps(Map.of(1, colour));
        rightL4Led = LEDPattern.steps(Map.of(1, Color.kBlack));

        // Solid and Blinking Patterns
        solid = LEDPattern.solid(colour);
        flashing = LEDPattern.solid(colour).blink(Seconds.of(0.5));

        leds.setData(ledBuffer);

        leds.start();
    }

    public void checkStates() {
        if (robotContainer.isCoral) {
            colour = Color.kWhite;
        } else {
            colour = Color.kAqua;
        }
        if (robotContainer.isLeft) {
            flashing.applyTo(leftLeds);
            solid.applyTo(rightLeds);
        } else {
            flashing.applyTo(rightLeds);
            solid.applyTo(rightLeds);
        }
    }

    public void L1() {
        checkStates();

    }

    public void L2() {
        checkStates();

    }

    public void L3() {
        checkStates();

    }

    public void L4() {
        checkStates();

    }

    public void Intake() {
        checkStates();

    }

    public void Outtake() {
        checkStates();

    }

    public void Score() {
    }

    public void LeftSide() {
    }

    public void RightSide() {
    }

    public void Stop() {
    }

    public void stopLED() {
        leds.stop();
    }
}
