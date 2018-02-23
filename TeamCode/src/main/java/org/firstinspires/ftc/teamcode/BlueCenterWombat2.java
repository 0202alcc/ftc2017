/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.util.VuforiaEncoder;

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@Autonomous(name="BlueCenterWombat2", group ="concept")
//@Disabled
public class BlueCenterWombat2 extends LinearOpMode {
    HardwareWombatTwo robot       = new HardwareWombatTwo();
    private ElapsedTime runtime = new ElapsedTime();
    private String susi;
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // Andymark Neverrest
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {
        //Init stuff
        robot.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        VuforiaEncoder ve = new VuforiaEncoder(hardwareMap, parameters);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftFrontMotor.getCurrentPosition(),
                robot.rightFrontMotor.getCurrentPosition());
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        ve.enable();
        ve.activate();
        while (!opModeIsActive()) {
            ve.track(telemetry);
            susi = ve.getLastVuMark();
        }
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("susi", susi);
            telemetry.update();
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
            drive(1, 23, 5.0);
            turnLeft(1.0, 5.0);
            strafe(1.0, false, 15, 5.0); //Strafe Left until aligned up perfectly against balancing stone
            if(susi == "LEFT"){
                telemetry.addData("LEFT", 0);
                strafe(1.0, true, 90/7, 5.0);
                drive(1, 15, 5.0);
                drive(1, -4, 5.0);
                robot.dump.setPosition(0.9);
                drive(1, -4, 5.0);
                drive(1, 10, 5.0);
                drive(1, -4, 5.0);
                //Strafe until lined up with left
            } else if(susi == "CENTER"){
                telemetry.addData("CENTER", 0);
                strafe(1.0, true, 20, 5.0);
                drive(1, 15, 5.0);
                drive(1, -4, 5.0);
                robot.dump.setPosition(0.9);
                drive(1, -4, 5.0);
                drive(1, 10, 5.0);
                drive(1, -4, 5.0);
                //Strafe until lined up with center
            } else if(susi == "RIGHT"){
                telemetry.addData("RIGHT", 0);
                strafe(1.0, true, 230/7, 5.0);
                drive(1, 15, 5.0);
                drive(1, -4, 5.0);
                robot.dump.setPosition(0.9);
                drive(1, -4, 5.0);
                drive(1, 10, 5.0);
                drive(1, -4, 5.0);
                //Strafe until lined up with right
            } else {
                telemetry.addData("DUNNO", 0);
                strafe(1.0, true, 90/7, 5.0);
                drive(1, 15, 5.0);
                drive(1, -4, 5.0);
                robot.dump.setPosition(0.9);
                drive(1, -4, 5.0);
                drive(1, 10, 5.0);
                drive(1, -4, 5.0);
                //Randomly choose one
            }
            telemetry.update();
            robot.juul.setPosition(0);
            sleep(30000);
        }
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
