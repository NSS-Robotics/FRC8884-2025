package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {

    private final AddressableLED leds;
    private final AddressableLEDBuffer ledBuffer;
    //private final AddressableLEDBufferView left = ledbuffer.createView(0, 0);
    //private final AddressableLEDBufferView right = ledbuffer.createView(0, 0);
    private LEDPattern colour;

    public LED() {
        leds = new AddressableLED(Constants.LEDConstants.channel);
        ledBuffer = new AddressableLEDBuffer(128);
        colour = LEDPattern.solid(Color.kAqua);
        leds.setLength(ledBuffer.getLength());
        colour.applyTo(ledBuffer);
        leds.setData(ledBuffer);
        leds.start();
    }

    public void startLED() {
        colour.applyTo(ledBuffer);
        leds.setData(ledBuffer);
    }

    public void halfLED() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, i > 64 ? 255 : 0, i > 64 ? 0 : 255, 0);
        }
        leds.setData(ledBuffer);
        leds.start();
    }

    @Override
    public void periodic() {
        colour.applyTo(ledBuffer);
        leds.setData(ledBuffer);
    }
}
