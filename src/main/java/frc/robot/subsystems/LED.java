package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotState;
import frc.robot.RobotContainer;
import java.util.Map;

public class LED extends SubsystemBase {

    private final AddressableLED leds;
    private final AddressableLEDBuffer ledBuffer;
    private final AddressableLEDBufferView leftFrontLeds;
    private final AddressableLEDBufferView leftBackLeds;
    private final AddressableLEDBufferView rightFrontLeds;
    private final AddressableLEDBufferView rightBackLeds;

    // Left and Right Patterns
    private LEDPattern leftFrontPattern = LEDPattern.solid(Color.kBlack);
    private LEDPattern leftBackPattern = LEDPattern.solid(Color.kBlack);
    private LEDPattern rightFrontPattern = LEDPattern.solid(Color.kBlack);
    private LEDPattern rightBackPattern = LEDPattern.solid(Color.kBlack);

    // Level Segments
    private LEDPattern leftL1Led;
    private LEDPattern rightL1Led;
    private LEDPattern leftL2Led;
    private LEDPattern rightL2Led;
    private LEDPattern leftL3Led;
    private LEDPattern rightL3Led;
    private LEDPattern leftL4Led;
    private LEDPattern rightL4Led;

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
            throw new IllegalArgumentException(
                "LED buffer length must be at least 154"
            );
        }

        leds.setLength(ledBuffer.getLength());

        rightBackLeds = ledBuffer.createView(0, 37);
        rightFrontLeds = ledBuffer.createView(38, 74).reversed();
        leftFrontLeds = ledBuffer.createView(75, 115);
        leftBackLeds = ledBuffer.createView(116, 153).reversed();

        leds.start();

        leftFrontPattern = LEDPattern.solid(Color.kViolet)
            .breathe(Seconds.of(3))
            .atBrightness(Percent.of(30));
        leftBackPattern = LEDPattern.solid(Color.kViolet)
            .breathe(Seconds.of(3))
            .atBrightness(Percent.of(30));
        rightFrontPattern = LEDPattern.solid(Color.kViolet)
            .breathe(Seconds.of(3))
            .atBrightness(Percent.of(30));
        rightBackPattern = LEDPattern.solid(Color.kViolet)
            .breathe(Seconds.of(3))
            .atBrightness(Percent.of(30));
    }

    private void checkGamepiece() {
        if (robotContainer.isCoral) {
            colour = Color.kAliceBlue;
        } else {
            colour = Color.kBlue;
        }
    }

    public void updateGamePiece() {
        checkGamepiece();
        // Check the current level and recall the corresponding level
        if (robotContainer.scoringLevel.equals(RobotState.l1)) {
            L1Leds();
        } else if (robotContainer.scoringLevel.equals(RobotState.l2)) {
            L2Leds();
        } else if (robotContainer.scoringLevel.equals(RobotState.l3)) {
            L3Leds();
        } else if (robotContainer.scoringLevel.equals(RobotState.l4)) {
            L4Leds();
        } else if (robotContainer.scoringLevel.equals(RobotState.processor)) {
            L1Leds();
        } else if (
            robotContainer.scoringLevel.equals(RobotState.algaeReefLow)
        ) {
            L2Leds();
        } else if (
            robotContainer.scoringLevel.equals(RobotState.algaeReefHigh)
        ) {
            L3Leds();
        } else if (robotContainer.scoringLevel.equals(RobotState.barge)) {
            L4Leds();
        }
    }

    private void setLeds(
        LEDPattern leftPatternToSet,
        LEDPattern rightPatternToSet
    ) {
        if (robotContainer.isCoral) {
            if (robotContainer.isLeft) {
                leftFrontPattern = leftPatternToSet.blink(Seconds.of(0.1));
                leftBackPattern = leftPatternToSet.blink(Seconds.of(0.1));
                rightFrontPattern = rightPatternToSet;
                rightBackPattern = rightPatternToSet;
            } else {
                rightFrontPattern = rightPatternToSet.blink(Seconds.of(0.1));
                rightBackPattern = rightPatternToSet.blink(Seconds.of(0.1));
                leftBackPattern = leftPatternToSet;
                leftFrontPattern = leftPatternToSet;
            }
        } else {
            leftFrontPattern = leftPatternToSet;
            leftBackPattern = leftPatternToSet;
            rightFrontPattern = rightPatternToSet;
            rightBackPattern = rightPatternToSet;
        }
    }

    public void L1Leds() {
        checkGamepiece();

        leftL1Led = LEDPattern.steps(
            Map.of(0, colour, 0.25, Color.kBlack)
        ).atBrightness(Percent.of(20));
        rightL1Led = LEDPattern.steps(
            Map.of(0, colour, 0.25, Color.kBlack)
        ).atBrightness(Percent.of(20));

        setLeds(leftL1Led, rightL1Led);
    }

    public void L2Leds() {
        checkGamepiece();

        leftL2Led = LEDPattern.steps(
            Map.of(0, colour, 0.5, Color.kBlack)
        ).atBrightness(Percent.of(20));
        rightL2Led = LEDPattern.steps(
            Map.of(0, colour, 0.5, Color.kBlack)
        ).atBrightness(Percent.of(20));

        setLeds(leftL2Led, rightL2Led);
    }

    public void L3Leds() {
        checkGamepiece();

        leftL3Led = LEDPattern.steps(
            Map.of(0, colour, 0.75, Color.kBlack)
        ).atBrightness(Percent.of(20));
        rightL3Led = LEDPattern.steps(
            Map.of(0, colour, 0.75, Color.kBlack)
        ).atBrightness(Percent.of(20));

        setLeds(leftL3Led, rightL3Led);
    }

    public void L4Leds() {
        checkGamepiece();

        leftL4Led = LEDPattern.solid(colour).atBrightness(Percent.of(20));
        rightL4Led = LEDPattern.solid(colour).atBrightness(Percent.of(20));

        setLeds(leftL4Led, rightL4Led);
    }

    public void intakeLeds() {
        colour = Color.kBlue;

        LEDPattern breathe = LEDPattern.solid(colour)
            .breathe(Seconds.of(.25))
            .atBrightness(Percent.of(20));

        leftFrontPattern = breathe;
        leftBackPattern = breathe;
        rightFrontPattern = breathe;
        rightBackPattern = breathe;
    }

    public void intakeCompleteLeds() {
        colour = Color.kViolet;

        LEDPattern strobe = LEDPattern.solid(colour)
            .blink(Seconds.of(0.2))
            .atBrightness(Percent.of(20));

        leftFrontPattern = strobe;
        leftBackPattern = strobe;
        rightFrontPattern = strobe;
        rightBackPattern = strobe;
    }

    public void outtakeLeds() {
        colour = Color.kCoral;
        colour2 = Color.kAliceBlue;

        LEDPattern gradientStrobe = LEDPattern.gradient(
            LEDPattern.GradientType.kDiscontinuous,
            colour,
            colour2
        )
            .blink(Seconds.of(0.1))
            .atBrightness(Percent.of(20));

        leftFrontPattern = gradientStrobe;
        leftBackPattern = gradientStrobe;
        rightFrontPattern = gradientStrobe;
        rightBackPattern = gradientStrobe;
    }

    public void alignLeds() {
        colour = Color.kLimeGreen;

        LEDPattern alignPattern = LEDPattern.solid(colour)
            .blink(Seconds.of(0.1))
            .atBrightness(Percent.of(20));

        leftFrontPattern = alignPattern;
        leftBackPattern = alignPattern;
        rightFrontPattern = alignPattern;
        rightBackPattern = alignPattern;
    }

    public void score(Elevator m_elevator) {
        // ! Change to use max position not rotations
        elevatorProgress = LEDPattern.progressMaskLayer(
            () ->
                m_elevator.getPosition() /
                Constants.ElevatorConstants.maxRotations
        ).atBrightness(Percent.of(20));

        leftFrontPattern = elevatorProgress;
        leftBackPattern = elevatorProgress;
        rightFrontPattern = elevatorProgress;
        rightBackPattern = elevatorProgress;
    }

    public void climbLeds() {
        colour = Color.kAliceBlue;
        colour2 = Color.kViolet;

        LEDPattern climbPattern = LEDPattern.gradient(
            LEDPattern.GradientType.kDiscontinuous,
            colour,
            colour2
        )
            .breathe(Seconds.of(0.5))
            .atBrightness(Percent.of(20));

        leftFrontPattern = climbPattern;
        leftBackPattern = climbPattern;
        rightFrontPattern = climbPattern;
        rightBackPattern = climbPattern;
    }

    public void stop() {
        LEDPattern off = LEDPattern.solid(Color.kBlack);

        leftFrontPattern = off;
        leftBackPattern = off;
        rightFrontPattern = off;
        rightBackPattern = off;
    }

    @Override
    public void periodic() {
        leftFrontPattern.applyTo(leftFrontLeds);
        leftBackPattern.applyTo(leftBackLeds);
        rightFrontPattern.applyTo(rightFrontLeds);
        rightBackPattern.applyTo(rightBackLeds);
        leds.setData(ledBuffer);
    }
}
