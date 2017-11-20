package org.firstinspires.ftc.teamcode.jewel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwarePushbotMecanum;

/**
 * Created by gescalona on 11/17/17.
 */
@TeleOp(name="sample op mode", group ="Pushbot")
public class jewelopmode extends OpMode {
    ElapsedTime time = new ElapsedTime();
    HardwarePushbotMecanum robot;
    private JewelSensorManager manager;
    public void init() {
        robot.init(hardwareMap);
        manager = new JewelSensorManager(robot.color);

    }
    public void loop(){
        telemetry.addData("Runtime", time);
        telemetry.addData("Red", manager.getRed());
    }
}
