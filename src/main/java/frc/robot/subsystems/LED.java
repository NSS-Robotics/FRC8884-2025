package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.Map;

public class LED extends SubsystemBase {

    private final AddressableLED leds;
    private final AddressableLEDBuffer ledBuffer;
    private final AddressableLEDBufferView leftFrontLeds;
    private final AddressableLEDBufferView leftBackLeds;
    private final AddressableLEDBufferView rightFrontLeds;
    private final AddressableLEDBufferView rightBackLeds;

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
    private LEDPattern breathe;
    private LEDPattern gradient;

    // Elevator Progress
    private LEDPattern elevatorProgress;

    private Color colour = Color.kViolet;
    private Color colour2;

    private RobotContainer robotContainer;

    public LED(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;

        leds = new AddressableLED(Constants.LEDConstants.channel);
        ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.length);

        if (ledBuffer.getLength() < 151) {
            throw new IllegalArgumentException("LED buffer length must be at least 151");
        }

        leds.setLength(ledBuffer.getLength());

        leftFrontLeds = ledBuffer.createView(0, 37).reversed();
        leftBackLeds = ledBuffer.createView(38, 75);
        rightFrontLeds = ledBuffer.createView(76, 112).reversed(); // Throws an error when it's more than 112 but I have
                                                                   // no idea why
        rightBackLeds = ledBuffer.createView(113, 150);

        // Solid and Blinking Patterns
        solid = LEDPattern.solid(colour);
        flashing = LEDPattern.solid(colour).blink(Seconds.of(0.5));
        breathe = LEDPattern.solid(colour).breathe(Seconds.of(3));
        gradient = LEDPattern.gradient(
                LEDPattern.GradientType.kDiscontinuous,
                colour,
                colour2);

        solid.applyTo(ledBuffer);

        leds.setData(ledBuffer);
        leds.start();

        L4Leds();

    }

    private void checkStates() {
        stop();
        if (robotContainer.isCoral) {
            colour = Color.kWhite;
        } else {
            colour = Color.kAqua;
        }
        if (robotContainer.isLeft) {
            flashing.applyTo(leftFrontLeds, leftBackLeds);
            solid.applyTo(rightFrontLeds, rightBackLeds);
            leds.setData(ledBuffer);
        } else {
            flashing.applyTo(rightFrontLeds, rightBackLeds);
            solid.applyTo(leftFrontLeds, leftBackLeds);
            leds.setData(ledBuffer);
        }
    }

    // private LEDPattern solid(Color colour) {
    // System.out.println(colour);
    // return LEDPattern.solid(colour);
    // }

    // private LEDPattern flash(Color colour) {
    // return LEDPattern.solid(colour).blink(Seconds.of(0.5));
    // }

    public void L1Leds() {
        checkStates();
        leftL1Led = LEDPattern.steps(
                Map.of(0, colour, 0.25, Color.kBlack, 0.75, colour));
        rightL1Led = LEDPattern.steps(
                Map.of(0, colour, 0.25, Color.kBlack, 0.75, colour));

        leftL1Led.applyTo(leftFrontLeds, leftBackLeds);
        rightL1Led.applyTo(rightFrontLeds, rightBackLeds);
        leds.setData(ledBuffer);
    }

    public void L2Leds() {
        checkStates();
        leftL2Led = LEDPattern.steps(
                Map.of(0, colour, 0.4, Color.kBlack, 0.6, colour));
        rightL2Led = LEDPattern.steps(
                Map.of(0, colour, 0.4, Color.kBlack, 0.6, colour));

        leftL2Led.applyTo(leftFrontLeds, leftBackLeds);
        rightL2Led.applyTo(rightFrontLeds, rightBackLeds);
        leds.setData(ledBuffer);
    }

    public void L3Leds() {
        checkStates();
        leftL3Led = LEDPattern.steps(
                Map.of(0, colour, 0.6, Color.kBlack, 0.7, colour));
        rightL3Led = LEDPattern.steps(
                Map.of(0, colour, 0.6, Color.kBlack, 0.7, colour));

        leftL3Led.applyTo(leftFrontLeds, leftBackLeds);
        rightL3Led.applyTo(rightFrontLeds, rightBackLeds);
        leds.setData(ledBuffer);
    }

    public void L4Leds() {
        checkStates();
        System.out.println(colour);
        leftL4Led = LEDPattern.steps(Map.of(0, colour, 1, colour));
        rightL4Led = LEDPattern.steps(Map.of(0, colour, 1, colour));

        leftL4Led.applyTo(leftFrontLeds, leftBackLeds);
        rightL4Led.applyTo(rightFrontLeds, rightBackLeds);
        leds.setData(ledBuffer);
    }

    public void intakeLeds() {
        colour = Color.kForestGreen;
        breathe.applyTo(ledBuffer);
        leds.setData(ledBuffer);
    }

    public void outtakeLeds() {
        colour = Color.kCoral;
        colour2 = Color.kRed;
        gradient.applyTo(ledBuffer);
        leds.setData(ledBuffer);
    }

    public void alignLeds() {
        colour = Color.kBlue;
        flashing.applyTo(ledBuffer);
        leds.setData(ledBuffer);
    }

    public void score(Elevator m_elevator) {
        // ! Change to use max position not rotations
        elevatorProgress = LEDPattern.progressMaskLayer(
                () -> m_elevator.getPosition() /
                        Constants.ElevatorConstants.maxRotations);

        elevatorProgress.applyTo(ledBuffer);
        leds.setData(ledBuffer);
    }

    public void climb() {
        colour = Color.kYellow;
        solid.applyTo(ledBuffer);
        leds.setData(ledBuffer);
    }

    public void stop() {
        LEDPattern.solid(Color.kBlack).applyTo(ledBuffer);
        leds.setData(ledBuffer);
    }
}
