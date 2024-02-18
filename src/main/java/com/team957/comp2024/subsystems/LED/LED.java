package com.team957.comp2024.subsystems.LED;

import java.lang.invoke.LambdaConversionException;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class LED implements Subsystem{
    
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    //private AddressableLEDBuffer m_ledSegment;

    int Pixel;
    int startPixel;
    int endPixel;
    int totalPixels;
    
    public LED(){
        register();

        m_led = new AddressableLED(0); 

        m_ledBuffer = new AddressableLEDBuffer(0);
        m_led.setLength(m_ledBuffer.getLength());
        
        int totalPixels = m_ledBuffer.getLength();

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    /*public int startPixel(int totalPixels){
        return totalPixels;
    }*/
    
    public void setStartAndEndPixel(){
        //For Loop to first define Start and End Pixels
        for (int i = 0; i <= totalPixels; i++){
            
        }
    }
}
