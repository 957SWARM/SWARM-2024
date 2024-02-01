package com.team957.comp2024.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED implements Subsystem{

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;

    public LED(){
        m_led = new AddressableLED(0); 

        m_ledBuffer = new AddressableLEDBuffer(0);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
        
    }
}
