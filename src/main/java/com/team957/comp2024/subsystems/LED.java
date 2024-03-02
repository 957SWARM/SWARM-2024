package com.team957.comp2024.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import monologue.Logged;

public class LED implements Subsystem, Logged {

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private AddressableLEDBuffer ledSegment;

    public LED() {
        led = new AddressableLED(0);

        ledBuffer = new AddressableLEDBuffer(0);
        led.setLength(ledBuffer.getLength());

        // Set the data
        led.setData(ledBuffer);
        led.start();
    }

    public Command segmentation() {
        // what is this supposed to do?
        // return this.runOnce(() -> m_ledSegment = ledSegment.get(ledBuffer));
        return null;
    }

    @Override
    public void periodic() {
        Command activeCommand = getCurrentCommand();

        if (activeCommand != null) log("activeCommand", activeCommand.getName());
    }
}
