package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogOutput;

public class AnalogLib {
    private AnalogInput aInput;
    private AnalogOutput aOutput;

    public AnalogLib(int inChannel, int outChannel) {
        aInput = new AnalogInput(inChannel);
        aOutput = new AnalogOutput(outChannel);
    }

    public void setVoltage(double value) {
        aOutput.setVoltage(value);
    }

    public double getVoltage() {
        return aInput.getVoltage();
    }

    public int getValue() {
        return aInput.getValue();
    }
}