package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

public class DigitalLib {
    private DigitalInput dInput;
    private DigitalOutput dOutput;

    public DigitalLib(int inChannel, int outChannel) {
        dInput = new DigitalInput(inChannel);
        dOutput = new DigitalOutput(outChannel);
    }

    public void pulse(double pulseLength) {
        dOutput.pulse(pulseLength);
    }

    public void set(boolean value) {
        dOutput.set(value);
    }

    public boolean get() {
        return dInput.get();
    }
}