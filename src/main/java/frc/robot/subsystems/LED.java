package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {

    private final AddressableLED leds = new AddressableLED(
        Constants.LEDConstants.channel
    );
    private final AddressableLEDBuffer ledbuffer = new AddressableLEDBuffer(
        100
    );
    private final AddressableLEDBufferView left = ledbuffer.createView(0, 0);
    private final AddressableLEDBufferView right = ledbuffer.createView(0, 0);
    private LEDPattern colour;

    public LED() {
        leds.start();
        colour = LEDPattern.solid(Color.kAqua);
        leds.setLength(ledbuffer.getLength());
        colour.applyTo(ledbuffer);
        leds.setData(ledbuffer);
    }

    public void startLED() {
        leds.setData(ledbuffer);
    }
}
