package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Pixy2 {

    public AnalogInput pixytrig = null;
    public AnalogInput pixyvolt = null;
    public double voltage = -100;

    public void initialize(HardwareMap hwmap, String tname, String vname) {
        pixytrig = hwmap.get(AnalogInput.class, vname);
        pixyvolt = hwmap.get(AnalogInput.class, tname);
    }

    public double getValue() {
        voltage = (pixytrig.getVoltage() > 1.5) ? pixyvolt.getVoltage() : -100;
        return voltage;
    }

}
