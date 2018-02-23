package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.util.VuforiaEncoder;

/**
 * Created by gescalona on 1/12/18.
 */
@Autonomous(name="Autonomous Test for IMU", group="Pushbot")
public class autotestmode extends LinearOpMode {
    @Override
    public void runOpMode(){
        VuforiaEncoder ve = null;
        ve.enable();
        waitForStart();
        ve.activate();
        while(opModeIsActive()){
        ve.track(telemetry);
        }
    }
}
