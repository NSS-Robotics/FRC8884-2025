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
    private LEDPattern reversedElevatorProgress;

    private Color colour;
    private Color colour2;

    private RobotContainer robotContainer;

    public LED(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;

        leds = new AddressableLED(Constants.LEDConstants.channel);
        ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.length);
        leds.setLength(ledBuffer.getLength());

        leftFrontLeds = new AddressableLEDBufferView(ledBuffer, 0, 37);
        leftBackLeds = new AddressableLEDBufferView(ledBuffer, 38, 75);
        rightFrontLeds = new AddressableLEDBufferView(ledBuffer, 76, 114);
        rightBackLeds = new AddressableLEDBufferView(ledBuffer, 114, 150);

        colour = Color.kViolet;

        // Solid and Blinking Patterns
        solid = LEDPattern.solid(colour);
        flashing = LEDPattern.solid(colour).blink(Seconds.of(0.5));
        breathe = LEDPattern.solid(colour).breathe(Seconds.of(3));
        gradient = LEDPattern.gradient(
            LEDPattern.GradientType.kDiscontinuous,
            colour,
            colour2
        );

        // breathe.applyTo(ledBuffer);
        solid.applyTo(ledBuffer);

        leds.setData(ledBuffer);
        // leds.start();
    }

    private void checkStates() {
        // if (robotContainer.isCoral) {
        //     colour = Color.kWhite;
        // } else {
        //     colour = Color.kAqua;
        // }
        // if (robotContainer.isLeft) {
        //     flashing.applyTo(leftFrontLeds, leftBackLeds);
        //     solid.applyTo(rightFrontLeds, rightBackLeds);
        // } else {
        //     flashing.applyTo(rightFrontLeds, rightBackLeds);
        //     solid.applyTo(leftFrontLeds, leftBackLeds);
        // }
    }

    public void L1Leds() {
        // checkStates();
        // leftL1Led = LEDPattern.steps(
        //     Map.of(0, colour, 0.25, Color.kBlack, 0.75, colour)
        // );
        // rightL1Led = LEDPattern.steps(
        //     Map.of(0, Color.kBlack, 0.25, colour, 0.75, Color.kBlack)
        // );

        // leftL1Led.applyTo(leftFrontLeds, leftBackLeds);
        // rightL1Led.applyTo(rightFrontLeds, rightBackLeds);
        // leds.setData(ledBuffer);
    }

    public void L2Leds() {
        // checkStates();
        // leftL2Led = LEDPattern.steps(
        //     Map.of(0, colour, 0.4, Color.kBlack, 0.6, colour)
        // );
        // rightL2Led = LEDPattern.steps(
        //     Map.of(0, Color.kBlack, 0.4, colour, 0.6, Color.kBlack)
        // );

        // leftL2Led.applyTo(leftFrontLeds, leftBackLeds);
        // rightL2Led.applyTo(rightFrontLeds, rightBackLeds);
        // leds.setData(ledBuffer);
    }

    public void L3Leds() {
        // checkStates();
        // leftL3Led = LEDPattern.steps(
        //     Map.of(0, colour, 0.6, Color.kBlack, 0.7, colour)
        // );
        // rightL3Led = LEDPattern.steps(
        //     Map.of(0, Color.kBlack, 0.6, colour, 0.7, Color.kBlack)
        // );

        // leftL3Led.applyTo(leftFrontLeds, leftBackLeds);
        // rightL3Led.applyTo(rightFrontLeds, rightBackLeds);
        // leds.setData(ledBuffer);
    }

    public void L4Leds() {
        // checkStates();
        // leftL4Led = LEDPattern.steps(Map.of(1, colour));
        // rightL4Led = LEDPattern.steps(Map.of(1, colour));

        // leftL4Led.applyTo(leftFrontLeds, leftBackLeds);
        // rightL4Led.applyTo(rightFrontLeds, rightBackLeds);
        // leds.setData(ledBuffer);
    }

    public void intakeLeds() {
        // colour = Color.kForestGreen;
        // breathe.applyTo(ledBuffer);
        // leds.setData(ledBuffer);
    }

    public void outtakeLeds() {
        // colour = Color.kCoral;
        // colour2 = Color.kRed;
        // gradient.applyTo(ledBuffer);
        // leds.setData(ledBuffer);
    }

    public void alignLeds() {
        // colour = Color.kBlue;
        // flashing.applyTo(ledBuffer);
        // leds.setData(ledBuffer);
    }

    public void score(Elevator m_elevator) {
        // ! Change to use max position not rotations
        // elevatorProgress = LEDPattern.progressMaskLayer(
        //     () ->
        //         m_elevator.getPosition() /
        //         Constants.ElevatorConstants.maxRotations
        // );

        // reversedElevatorProgress = elevatorProgress.reversed();

        // elevatorProgress.applyTo(leftBackLeds, rightBackLeds);
        // reversedElevatorProgress.applyTo(leftFrontLeds, rightFrontLeds);
        // leds.setData(ledBuffer);
    }

    public void climb() {
        // colour = Color.kYellow;
        // solid.applyTo(ledBuffer);
        // leds.setData(ledBuffer);
    }

    public void stop() {
        // colour = Color.kBlack;
        // solid.applyTo(ledBuffer);
        // leds.setData(ledBuffer);
    }
}
