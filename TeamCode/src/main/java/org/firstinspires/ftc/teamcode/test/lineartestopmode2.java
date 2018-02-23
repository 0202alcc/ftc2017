package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Angler;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by gescalona on 1/12/18.
 */
@TeleOp(name="Linear Test OP mode", group="Pushbot")
public class lineartestopmode2 extends OpMode {
    private HardwareTest robot = new HardwareTest();
    private Angler angler;
    private double HEADING_THRESHOLD = 1;
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;
    @Override
    public void init(){
        robot.init(hardwareMap);
        angler = new Angler(hardwareMap,telemetry);
        angler.start();
        angler.composeTelemetry();
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftBack");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        rightBackMotor = hardwareMap.dcMotor.get("rightBack");
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
    public void makeTurn(double speed, double angle){
        int rangle = Integer.parseInt(angler.getAngle());
        int good = fixAngle(rangle);
        int targetangle = good + rangle;
        double leftspeed,rightspeed;
        List<DcMotor> left = new ArrayList<DcMotor>();
        left.add(leftBackMotor);
        left.add(leftFrontMotor);
        List<DcMotor> right = new ArrayList<DcMotor>();
        right.add(leftBackMotor);
        right.add(leftFrontMotor);
        if(angle > 180) {
            leftspeed = 1;
            rightspeed =-1;
        }else{
            leftspeed = -1;
            rightspeed = 1;
        }
        while(!(targetangle + 5 >=  Integer.parseInt(angler.getAngle()))) {
            for (DcMotor l : left) {
                l.setPower(leftspeed);
            }
            for (DcMotor r : right) {
                r.setPower(rightspeed);
            }
        }
        for (DcMotor l : left) {
            l.setPower(0);
        }
        for (DcMotor r : right) {
            r.setPower(0);
        }

    }
    public int fixAngle(int angle){
        if(angle < 0){
            angle = angle + 360;
        }
        return angle; // for now
    }
    /*public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }*/
    /*boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftDrive.setPower(leftSpeed);
        robot.rightDrive.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }*/

}