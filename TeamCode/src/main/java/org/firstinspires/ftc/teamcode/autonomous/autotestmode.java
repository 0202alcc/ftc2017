package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by gescalona on 1/12/18.
 */
@Autonomous(name="Autonomous Test OP mode", group="Pushbot")
public class autotestmode extends LinearOpMode {
    @Override
    public void runOpMode(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        VuforiaEncoder ve = new VuforiaEncoder(hardwareMap, parameters);
        ve.enable();
        waitForStart();
        ve.activate();
        while(opModeIsActive()){
        ve.track(telemetry);
        }
    }
}
