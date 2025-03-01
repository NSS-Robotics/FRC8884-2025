package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LED extends SubsystemBase {
    // private final AddressableLED leds;
    // private final AddressableLEDBuffer ledBuffer;
    // private final AddressableLEDBufferView left;
    // private final AddressableLEDBufferView right;
    // private LEDPattern colour = LEDPattern.solid(Color.kCrimson);

    // private RobotContainer robotContainer;

    // public LED(RobotContainer robotContainer) {
    //     this.robotContainer = robotContainer;

    //     leds = new AddressableLED(Constants.LEDConstants.channel);
    //     ledBuffer = new AddressableLEDBuffer(151);
    //     leds.setLength(ledBuffer.getLength());
    //     leds.setData(ledBuffer);
    //     leds.start();
    //     left = ledBuffer.createView(0, ledBuffer.getLength() / 2);
    //     right = ledBuffer.createView(
    //         ledBuffer.getLength() / 2,
    //         ledBuffer.getLength()
    //     );
    // }

    // // Turns on all the LEDs, just for testing.
    // public void testLED() {
    //     colour.applyTo(ledBuffer);
    // }

    // public void runLED() {
    //     leds.start();
    //     if (robotContainer.isCoral) {
    //         colour = LEDPattern.solid(Color.kWhite);
    //     } else {
    //         colour = LEDPattern.solid(Color.kAqua);
    //     }

    //     // FIXME: We have to check if we have a coral/algae in our robot before turning on lights

    //     if (robotContainer.isLeft) {
    //         colour.applyTo(left);
    //     } else {
    //         colour.applyTo(right);
    //     }
    // }

    // public void stopLED() {
    //     leds.stop();
    // }
}
