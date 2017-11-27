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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: Auto Drive By Gyro", group="Pushbot")
@Disabled
public class MecanumTrigDriveAuto extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbotMecanum robot       = new HardwarePushbotMecanum();   // Use a Pushbot's hardware
    ModernRoboticsI2cGyro   gyro    = null;                    // Additional Gyro device

    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * Math.PI);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 1;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 1;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable


    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
        }

        gyro.resetZAxisIntegrator();
        // Use GYRODRIVE FOR REGULAR MOTION
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        gyroDrive(DRIVE_SPEED, 48.0, 0.0);    // Drive FWD 48 inches
        gyroTurn( TURN_SPEED, -45.0);         // Turn  CCW to -45 Degrees


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    // distance is east = positive value and west = negative value
    public void mecanumDrive (double angle, double distance, double timeout){
        int newv1v4Target;
        int newv2v3Target;
        int moveCounts;
        double max;
        double error;

        double robotAngle = angle - Math.PI/4;
        moveCounts = (int)(distance * COUNTS_PER_INCH);
        newv1v4Target = robot.leftFrontMotor.getCurrentPosition() + moveCounts;
        newv2v3Target = robot.rightFrontMotor.getCurrentPosition() - moveCounts;

        robot.leftFrontMotor.setTargetPosition(newv1v4Target);
        robot.rightBackMotor.setTargetPosition(newv1v4Target);
        robot.rightFrontMotor.setTargetPosition(newv2v3Target);
        robot.leftBackMotor.setTargetPosition(newv2v3Target);

        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        final double v1 = Math.cos(robotAngle);
        final double v2 = Math.sin(robotAngle);
        final double v3 = Math.sin(robotAngle);
        final double v4 = Math.cos(robotAngle);

        robot.leftFrontMotor.setPower(v1);
        robot.rightFrontMotor.setPower(v2);
        robot.leftBackMotor.setPower(v3);
        robot.rightBackMotor.setPower(v4);
    }

   /**
    *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
    * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from current heading.
    */
    public void gyroDrive (double speed, double distance, double angle){
       int     newLeftTarget;
       int     newRightTarget;
       int     moveCounts;
       double  max;
       double  error;
       double  steer;
       double  leftSpeed;
       double  rightSpeed;

       if (opModeIsActive()) {
           //Determine target position
           moveCounts = (int)(distance * COUNTS_PER_INCH);
           newLeftTarget = robot.leftBackMotor.getCurrentPosition() + moveCounts;
           newRightTarget = robot.rightBackMotor.getCurrentPosition() + moveCounts;

           //Set Target and activate RUN_TO_POSITION
           robot.leftBackMotor.setTargetPosition(newLeftTarget);
           robot.leftFrontMotor.setTargetPosition(newLeftTarget);

           robot.rightBackMotor.setTargetPosition(newRightTarget);
           robot.rightFrontMotor.setTargetPosition(newRightTarget);

           robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

           //start motion
           speed = Range.clip(Math.abs(speed), 0.0, 1.0);
           robot.leftFrontMotor.setPower(speed);
           robot.leftBackMotor.setPower(speed);
           robot.rightBackMotor.setPower(speed);
           robot.rightFrontMotor.setPower(speed);

           //Keep looping while both motors are active
           while (opModeIsActive() && (robot.leftFrontMotor.isBusy() && robot.leftBackMotor.isBusy()) && (robot.rightFrontMotor.isBusy() && robot.rightBackMotor.isBusy())){
               //adjust speed on account of error
               error = getError(angle);
               steer = getSteer(error, P_DRIVE_COEFF);

               //motor correction must be reversed for driving in reverse
               if (distance < 0)
                   steer *= -1.0;

               leftSpeed = speed - steer;
               rightSpeed = speed - steer;

               //normalize speeds
               max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
               if (max > 1.0){
                   leftSpeed /= max;
                   rightSpeed /= max;
               }
               robot.leftFrontMotor.setPower(leftSpeed);
               robot.leftBackMotor.setPower(leftSpeed);
               robot.rightFrontMotor.setPower(rightSpeed);
               robot.rightBackMotor.setPower(rightSpeed);

               //Display driver status
               telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
               telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
               telemetry.addData("Actual",  "%7d:%7d",      robot.leftBackMotor.getCurrentPosition(), robot.rightBackMotor.getCurrentPosition());
               telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
               telemetry.update();
           }

           //Stop
           robot.leftFrontMotor.setPower(0);
           robot.leftBackMotor.setPower(0);
           robot.rightBackMotor.setPower(0);
           robot.rightFrontMotor.setPower(0);

           //Turn off RUN_TO_POSITION
           robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       }

   }
    public void gyroTurn (double speed, double angle){
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)){
            telemetry.update();
        }
    }
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftFrontMotor.setPower(0);
        robot.leftBackMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.rightBackMotor.setPower(0);
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
        robot.leftFrontMotor.setPower(leftSpeed);
        robot.leftBackMotor.setPower(leftSpeed);
        robot.rightFrontMotor.setPower(rightSpeed);
        robot.rightBackMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}
