package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem {
    AddressableLED m_led = new AddressableLED(LEDConstants.kCanID);
    AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LEDConstants.kStripLength);

    public LEDSubsystem() {
        m_led.setLength(LEDConstants.kStripLength);
        m_led.setData(m_ledBuffer);
    }

    public void setAll(Color color) {
        for (int i = 0; i < LEDConstants.kStripLength; i++)
            m_ledBuffer.setLED(i, color);
        m_led.start();
        m_led.setData(m_ledBuffer);
        m_led.stop();
    }
}
