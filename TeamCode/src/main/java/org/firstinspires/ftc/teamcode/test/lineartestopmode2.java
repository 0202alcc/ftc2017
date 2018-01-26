package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.HardwarePushbotMecanum;
import org.firstinspires.ftc.teamcode.util.Angler;

import java.util.Locale;

/**
 * Created by gescalona on 1/12/18.
 */
@TeleOp(name="Linear Test OP mode", group="Pushbot")
public class lineartestopmode2 extends OpMode {
    private HardwareTest robot = new HardwareTest();
    private Angler angler;
    @Override
    public void init(){
        robot.init(hardwareMap);
        angler = new Angler(hardwareMap,telemetry);
        angler.start();
        angler.composeTelemetry();
    }
    @Override
    public void loop(){
        telemetry.addData("TEST WITH RADIAN:", angler.getAngle());
    }
    /*public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }*/
    public void makeTurn(){
        int angle = Integer.parseInt(angler.getAngle());
    }
}