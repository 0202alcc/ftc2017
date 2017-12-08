package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwarePushbotMecanum;

/**
 * Created by gescalona on 12/8/17.
 */
@Autonomous(name="ColorSensor TEst", group="Autonomous")
public class RedSensor extends LinearOpMode {
    static final double     P_TURN_COEFF            = 0.1;
    static final double     HEADING_THRESHOLD       = 1 ;
    HardwarePushbotMecanum robot = new HardwarePushbotMecanum();
    ColorSensor sensor = robot.sensor;
    Servo colorservo = robot.colorservo;
    ModernRoboticsI2cGyro gyro = robot.gyro;
    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        colorservo.setPosition(1);
        if(checkRed()){
            telemetry.addData("Sensing", "REd");
            gyroTurn(6, 0.5);
            gyroTurn(-6, 0.5);
        }else{
            /*telemetry.addData("Sensing","Blue");
            gyroTurn(-6, 0.5);
            gyroTurn(6, 0.5);*/
        }
        colorservo.setPosition(0);
        gyroTurn(90, 0.75);

    }
    public boolean checkRed(){
        if (sensor.red() > 3) {
            return true;
        }
        return false;// Assume nothing
    }
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }
    boolean onHeading(double speed, double angle, double PCoeff) {
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
        robot.leftBackMotor.setPower(leftSpeed);
        robot.leftBackMotor.setPower(leftSpeed);
        robot.rightBackMotor.setPower(rightSpeed);
        robot.rightBackMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", angle);
        telemetry.addData("Err/St", error);
        telemetry.addData("Speed.", leftSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}
