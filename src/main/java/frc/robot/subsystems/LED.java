package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotState;

import java.util.Map;

public class LED extends SubsystemBase {

    private final AddressableLED leds;
    private final AddressableLEDBuffer ledBuffer;
    private final AddressableLEDBufferView leftFrontLeds;
    private final AddressableLEDBufferView leftBackLeds;
    private final AddressableLEDBufferView rightFrontLeds;
    private final AddressableLEDBufferView rightBackLeds;

    // Left and Right Patterns
    private LEDPattern leftPattern = LEDPattern.solid(Color.kBlack);
    private LEDPattern rightPattern = LEDPattern.solid(Color.kBlack);

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

    private Color colour = Color.kAliceBlue;
    private Color colour2 = Color.kAquamarine;

    private RobotContainer robotContainer;

    public LED(RobotContainer robotContainer) {

        this.robotContainer = robotContainer;

        leds = new AddressableLED(Constants.LEDConstants.channel);
        ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.length);

        if (ledBuffer.getLength() < 154) {
            throw new IllegalArgumentException("LED buffer length must be at least 154");
        }

        leds.setLength(ledBuffer.getLength());

        rightBackLeds = ledBuffer.createView(0, 37);
        rightFrontLeds = ledBuffer.createView(38, 75).reversed();
        leftFrontLeds = ledBuffer.createView(76, 115).reversed();
        leftBackLeds = ledBuffer.createView(116, 153);

        // Solid and Blinking Patterns
        solid = LEDPattern.solid(colour).atBrightness(Percent.of(20));
        flashing = LEDPattern.solid(colour).blink(Seconds.of(0.5)).atBrightness(Percent.of(20));
        breathe = LEDPattern.solid(colour).breathe(Seconds.of(3)).atBrightness(Percent.of(20));
        gradient = LEDPattern.gradient(
                LEDPattern.GradientType.kDiscontinuous,
                colour,
                colour2).atBrightness(Percent.of(20));

        leds.start();

    }

    private void checkGamepiece() {
        if (robotContainer.isCoral) {
            colour = Color.kAliceBlue;
        } else {
            colour = Color.kAquamarine;
        }
    }

    public void updateGamePiece() {
        checkGamepiece();
        // Check the current level and recall the corresponding level
        if (robotContainer.scoringLevel == RobotState.l1) {
            L1Leds();
        } else if (robotContainer.scoringLevel == RobotState.l2) {
            L2Leds();
        } else if (robotContainer.scoringLevel == RobotState.l3) {
            L3Leds();
        } else if (robotContainer.scoringLevel == RobotState.l4) {
            L4Leds();
        } else if (robotContainer.scoringLevel == RobotState.processor) {
            L1Leds();
        } else if (robotContainer.scoringLevel == RobotState.algaeReefLow) {
            L2Leds();
        } else if (robotContainer.scoringLevel == RobotState.algaeReefHigh) {
            L3Leds();
        } else if (robotContainer.scoringLevel == RobotState.barge) {
            L4Leds();
        }
    }

    private void setLeds(LEDPattern leftPatternToSet, LEDPattern rightPatternToSet) {
        if (robotContainer.isLeft) {
            leftPattern = leftPatternToSet.blink(Seconds.of(0.2));
            rightPattern = rightPatternToSet;
        } else {
            rightPattern = rightPatternToSet.blink(Seconds.of(0.2));
            leftPatternToSet = leftPattern;

        }
    }

    public void L1Leds() {
        checkGamepiece();

        leftL1Led = LEDPattern.steps(
                Map.of(0, colour, 0.25, Color.kBlack, 0.75, colour));
        rightL1Led = LEDPattern.steps(
                Map.of(0, colour, 0.25, Color.kBlack, 0.75, colour));

        setLeds(leftL1Led, rightL1Led);
    }

    public void L2Leds() {
        checkGamepiece();

        leftL2Led = LEDPattern.steps(
                Map.of(0, colour, 0.4, Color.kBlack, 0.6, colour));
        rightL2Led = LEDPattern.steps(
                Map.of(0, colour, 0.4, Color.kBlack, 0.6, colour));

        setLeds(leftL2Led, rightL2Led);
    }

    public void L3Leds() {
        checkGamepiece();

        leftL3Led = LEDPattern.steps(
                Map.of(0, colour, 0.6, Color.kBlack, 0.7, colour));
        rightL3Led = LEDPattern.steps(
                Map.of(0, colour, 0.6, Color.kBlack, 0.7, colour));

        setLeds(leftL3Led, rightL3Led);
    }

    public void L4Leds() {
        checkGamepiece();

        leftL4Led = LEDPattern.steps(Map.of(0, colour, 1, colour));
        rightL4Led = LEDPattern.steps(Map.of(0, colour, 1, colour));

        setLeds(leftL4Led, rightL4Led);
    }

    public void intakeLeds() {
        colour = Color.kForestGreen;
        leftPattern = breathe;
        rightPattern = breathe;
    }

    public void outtakeLeds() {
        colour = Color.kCoral;
        colour2 = Color.kRed;

        leftPattern = gradient;
        rightPattern = gradient;
    }

    public void alignLeds() {
        leftPattern = flashing;
        rightPattern = flashing;
    }

    public void score(Elevator m_elevator) {
        // ! Change to use max position not rotations
        elevatorProgress = LEDPattern.progressMaskLayer(
                () -> m_elevator.getPosition() /
                        Constants.ElevatorConstants.maxRotations);

        elevatorProgress.applyTo(ledBuffer);
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

    @Override
    public void periodic() {
        leftPattern.applyTo(leftFrontLeds);
        leftPattern.applyTo(leftBackLeds);
        rightPattern.applyTo(rightFrontLeds);
        rightPattern.applyTo(rightBackLeds);
        leds.setData(ledBuffer);
    }
}
