package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.map.HardwareWombatTwo;

/**
 * Created by acandidato on 2/14/18.
 */
@Autonomous(name="Strafe Test", group="Pushbot")
public class StrafeTest extends LinearOpMode {
    HardwareWombatTwo robot       = new HardwareWombatTwo();
    private ElapsedTime     runtime = new ElapsedTime();

    public static final String TAG = "Vuforia VuMark Sample";

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // Andymark Neverrest
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    OpenGLMatrix lastLocation = null;
    String susi = "Undefined";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
    @Override
    public void runOpMode(){
        //Init stuff
        robot.init(hardwareMap);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftFrontMotor.getCurrentPosition(),
                robot.rightFrontMotor.getCurrentPosition());
        telemetry.update();

        waitForStart();
        robot.juul.setPosition(0);
        robot.dump.setPosition(0.5);
        robot.bat.setPosition(0.8);
        sleep(1000);
        if(robot.colorSensor.red() > robot.colorSensor.blue()){
            //DRIVE BACK THEN FORWARD
            colorTurnLeft(0.5, 5);
            sleep(500);
            robot.juul.setPosition(1);
            colorTurnRight(0.5, 5);

        } else {
            colorTurnRight(0.5, 5);
            sleep(500);
            robot.juul.setPosition(1);
            colorTurnLeft(0.5, 5);
        }
        drive(1, 20, 5.0);
        turnLeft(1.0, 5.0);
        strafe(1.0, false, 15, 5.0);
//        strafe(1.0, true, 90/7, 5.0);
    }
    public void colorTurnLeft(double speed, double timeout){
        int newLeftTarget;
        int newRightTarget;
        int newLeftBottomTarget;
        int newRightBottomTarget;
        if(opModeIsActive()){
            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftFrontMotor.getCurrentPosition() + (int)(5 * COUNTS_PER_INCH);
            newLeftBottomTarget = robot.leftBackMotor.getCurrentPosition() + (int)(5 * COUNTS_PER_INCH);
            newRightTarget = robot.rightFrontMotor.getCurrentPosition() + (int)(-5 * COUNTS_PER_INCH);
            newRightBottomTarget = robot.rightBackMotor.getCurrentPosition() + (int)(-5 * COUNTS_PER_INCH);
            robot.leftFrontMotor.setTargetPosition(newLeftTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBottomTarget);
            robot.rightFrontMotor.setTargetPosition(newRightTarget);
            robot.rightBackMotor.setTargetPosition(newRightBottomTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed));
            robot.leftBackMotor.setPower(Math.abs(speed));
            robot.rightBackMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            robot.rightFrontMotor.setPower(0);
            robot.rightBackMotor.setPower(0);
            robot.leftFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void colorTurnRight(double speed, double timeout){
        int newLeftTarget;
        int newRightTarget;
        int newLeftBottomTarget;
        int newRightBottomTarget;
        if(opModeIsActive()){
            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftFrontMotor.getCurrentPosition() + (int)(-5 * COUNTS_PER_INCH);
            newLeftBottomTarget = robot.leftBackMotor.getCurrentPosition() + (int)(-5 * COUNTS_PER_INCH);
            newRightTarget = robot.rightFrontMotor.getCurrentPosition() + (int)(5 * COUNTS_PER_INCH);
            newRightBottomTarget = robot.rightBackMotor.getCurrentPosition() + (int)(5 * COUNTS_PER_INCH);
            robot.leftFrontMotor.setTargetPosition(newLeftTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBottomTarget);
            robot.rightFrontMotor.setTargetPosition(newRightTarget);
            robot.rightBackMotor.setTargetPosition(newRightBottomTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed));
            robot.leftBackMotor.setPower(Math.abs(speed));
            robot.rightBackMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            robot.rightFrontMotor.setPower(0);
            robot.rightBackMotor.setPower(0);
            robot.leftFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void turnLeft(double speed, double timeout){
        int newLeftTarget;
        int newRightTarget;
        int newLeftBottomTarget;
        int newRightBottomTarget;
        if(opModeIsActive()){
            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftFrontMotor.getCurrentPosition() + (int)(20 * COUNTS_PER_INCH);
            newLeftBottomTarget = robot.leftBackMotor.getCurrentPosition() + (int)(20 * COUNTS_PER_INCH);
            newRightTarget = robot.rightFrontMotor.getCurrentPosition() + (int)(-20 * COUNTS_PER_INCH);
            newRightBottomTarget = robot.rightBackMotor.getCurrentPosition() + (int)(-20 * COUNTS_PER_INCH);
            robot.leftFrontMotor.setTargetPosition(newLeftTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBottomTarget);
            robot.rightFrontMotor.setTargetPosition(newRightTarget);
            robot.rightBackMotor.setTargetPosition(newRightBottomTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed));
            robot.leftBackMotor.setPower(Math.abs(speed));
            robot.rightBackMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            robot.rightFrontMotor.setPower(0);
            robot.rightBackMotor.setPower(0);
            robot.leftFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void turnRight(double speed, double timeout){
        int newLeftTarget;
        int newRightTarget;
        int newLeftBottomTarget;
        int newRightBottomTarget;
        if(opModeIsActive()){
            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftFrontMotor.getCurrentPosition() + (int)(-20 * COUNTS_PER_INCH);
            newLeftBottomTarget = robot.leftBackMotor.getCurrentPosition() + (int)(-20 * COUNTS_PER_INCH);
            newRightTarget = robot.rightFrontMotor.getCurrentPosition() + (int)(20 * COUNTS_PER_INCH);
            newRightBottomTarget = robot.rightBackMotor.getCurrentPosition() + (int)(20 * COUNTS_PER_INCH);
            robot.leftFrontMotor.setTargetPosition(newLeftTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBottomTarget);
            robot.rightFrontMotor.setTargetPosition(newRightTarget);
            robot.rightBackMotor.setTargetPosition(newRightBottomTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed));
            robot.leftBackMotor.setPower(Math.abs(speed));
            robot.rightBackMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            robot.rightFrontMotor.setPower(0);
            robot.rightBackMotor.setPower(0);
            robot.leftFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void drive(double speed, double inches, double timeout){
        int newLeftTarget;
        int newRightTarget;
        int newLeftBottomTarget;
        int newRightBottomTarget;
        if(opModeIsActive()){
            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftFrontMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newLeftBottomTarget = robot.leftBackMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightTarget = robot.rightFrontMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightBottomTarget = robot.rightBackMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            robot.leftFrontMotor.setTargetPosition(newLeftTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBottomTarget);
            robot.rightFrontMotor.setTargetPosition(newRightTarget);
            robot.rightBackMotor.setTargetPosition(newRightBottomTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed));
            robot.leftBackMotor.setPower(Math.abs(speed));
            robot.rightBackMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            robot.rightFrontMotor.setPower(0);
            robot.rightBackMotor.setPower(0);
            robot.leftFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // for boolean the ***false is left*** and ***true is right***
    public void strafe(double speed, boolean angle, double inches, double timeout){
        int newLeftTarget;
        int newRightTarget;
        int newLeftBottomTarget;
        int newRightBottomTarget;
        if (opModeIsActive()) {
            if (angle) {
                //strafe right
                // Determine new target position, and pass to motor controller
                newLeftTarget = robot.leftFrontMotor.getCurrentPosition() + (int)(Math.abs(inches) * COUNTS_PER_INCH);
                newLeftBottomTarget = robot.leftBackMotor.getCurrentPosition() + (int)(-Math.abs(inches) * COUNTS_PER_INCH);
                newRightTarget = robot.rightFrontMotor.getCurrentPosition() + (int)(-Math.abs(inches) * COUNTS_PER_INCH);
                newRightBottomTarget = robot.rightBackMotor.getCurrentPosition() + (int)(Math.abs(inches) * COUNTS_PER_INCH);
                robot.leftFrontMotor.setTargetPosition(newLeftTarget);
                robot.leftBackMotor.setTargetPosition(newLeftBottomTarget);
                robot.rightFrontMotor.setTargetPosition(newRightTarget);
                robot.rightBackMotor.setTargetPosition(newRightBottomTarget);

                // Turn On RUN_TO_POSITION
                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                robot.leftFrontMotor.setPower(Math.abs(speed));
                robot.rightFrontMotor.setPower(Math.abs(speed));
                robot.leftBackMotor.setPower(Math.abs(speed));
                robot.rightBackMotor.setPower(Math.abs(speed));

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.

                while (opModeIsActive() &&
                        (runtime.seconds() < timeout) &&
                        (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                    telemetry.addData("Path2",  "Running at %7d :%7d",
                            robot.leftFrontMotor.getCurrentPosition(),
                            robot.rightFrontMotor.getCurrentPosition(), robot.leftBackMotor.getCurrentPosition(), robot.rightBackMotor.getCurrentPosition());
                    telemetry.update();
                }
                // Stop all motion;
                robot.rightFrontMotor.setPower(0);
                robot.rightBackMotor.setPower(0);
                robot.leftFrontMotor.setPower(0);
                robot.leftBackMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            } else if (!angle) {
                //strafe left
                // Determine new target position, and pass to motor controller
                newLeftTarget = robot.leftFrontMotor.getCurrentPosition() + (int)(-Math.abs(inches) * COUNTS_PER_INCH);
                newLeftBottomTarget = robot.leftBackMotor.getCurrentPosition() + (int)(Math.abs(inches) * COUNTS_PER_INCH);
                newRightTarget = robot.rightFrontMotor.getCurrentPosition() + (int)(Math.abs(inches) * COUNTS_PER_INCH);
                newRightBottomTarget = robot.rightBackMotor.getCurrentPosition() + (int)(-Math.abs(inches) * COUNTS_PER_INCH);
                robot.leftFrontMotor.setTargetPosition(newLeftTarget);
                robot.leftBackMotor.setTargetPosition(newLeftBottomTarget);
                robot.rightFrontMotor.setTargetPosition(newRightTarget);
                robot.rightBackMotor.setTargetPosition(newRightBottomTarget);

                // Turn On RUN_TO_POSITION
                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                robot.leftFrontMotor.setPower(Math.abs(speed));
                robot.rightFrontMotor.setPower(Math.abs(speed));
                robot.leftBackMotor.setPower(Math.abs(speed));
                robot.rightBackMotor.setPower(Math.abs(speed));

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.

                while (opModeIsActive() &&
                        (runtime.seconds() < timeout) &&
                        (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                    telemetry.addData("Path2",  "Running at %7d :%7d",
                            robot.leftFrontMotor.getCurrentPosition(),
                            robot.rightFrontMotor.getCurrentPosition(), robot.leftBackMotor.getCurrentPosition(), robot.rightBackMotor.getCurrentPosition());
                    telemetry.update();
                }
                // Stop all motion;
                robot.rightFrontMotor.setPower(0);
                robot.rightBackMotor.setPower(0);
                robot.leftFrontMotor.setPower(0);
                robot.leftBackMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }
}
