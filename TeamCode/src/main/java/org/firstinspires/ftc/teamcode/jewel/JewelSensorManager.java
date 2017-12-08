package org.firstinspires.ftc.teamcode.jewel;

import com.qualcomm.robotcore.hardware.ColorSensor;

public class JewelSensorManager {
    private ColorSensor sensor = null;
    public JewelSensorManager(ColorSensor cs){
        sensor = cs;
    }
    public int getRed(){
        return sensor.red();
    }
}
