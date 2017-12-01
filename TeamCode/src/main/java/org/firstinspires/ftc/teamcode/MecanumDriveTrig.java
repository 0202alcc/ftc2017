/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Mat;

@TeleOp(name="MecanumDriveTrig", group="Pushbot")
//@Disabled
public class MecanumDriveTrig extends OpMode {

    /* Declare OpMode members. */
    HardwarePushbotMecanum robot       = new HardwarePushbotMecanum();   // Use a Pushbot's hardware
    public ElapsedTime runtime = new ElapsedTime();
    boolean foo = false;
    double dumpx;
    double intakeValueLeft;
    double intakeValueRight;
    int liftCount = 0;
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: Anymark Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 0.8188976 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        robot.rackAndPinion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Path0",  "Starting at  :", robot.rackAndPinion.getCurrentPosition());
        telemetry.addData("Say", "Hello and, again, welcome to the Aperture Science computer-aided enrichment center.");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //gamepad2 uses the analog sticks to determine the power of each of the servos
        intakeValueLeft = ((-gamepad2.left_stick_y) + 1) / 2;
        robot.leftContinuous.setPosition(intakeValueLeft);

        intakeValueRight = ((-gamepad2.right_stick_y) + 1) / 2;
        robot.rightContinuous.setPosition(-intakeValueRight);

        // left bumper lowers lift, right bumper lifts lift, right triger controls servo
        if (gamepad1.left_bumper){
            encoderDrive(0.5, -6, 10);
            liftCount -= 6;
        }
        if (gamepad1.right_bumper){
            encoderDrive(0.5, 6, 10);
            liftCount += 6;
        }
        dumpx = gamepad1.right_trigger;
        if(dumpx < 1.0 && dumpx > 0){
            robot.dump.setPosition(0.1); //Confirm Servo Position
        }
        while(dumpx == 1.0){
            robot.dump.setPosition(1.0); //Confirm Servo Position
        }
        /*
         * This is the Mecanum Drive part. The math is explained in the engineering notebook.
         */
        //Thank you reddit user u/2treecko
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        // how much to amplify the power
        double r = Math.hypot(y, x);
        //calculates the angle of the joystick - 45 degrees
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;
        // rotation
        double rightX = -gamepad1.right_stick_x;
        // equation for each of the wheels
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        robot.leftFrontMotor.setPower(v1);
        robot.rightFrontMotor.setPower(v2);
        robot.leftBackMotor.setPower(v3);
        robot.rightBackMotor.setPower(v4);

        //extra omni power
            //Quad I
        if(x > 0 && y > 0){
            robot.leftOmni.setPower(v1);
            robot.rightOmni.setPower(v1);
        }
            //Quad II
        if(x < 0 && y > 0){
            robot.leftOmni.setPower(v2);
            robot.rightOmni.setPower(v2);
        }
            //Quad III
        if(x < 0 && y < 0){
            robot.leftOmni.setPower(v1);
            robot.rightOmni.setPower(v1);
        }
            //Quad IV
        if(x > 0 && y < 0){
            robot.leftOmni.setPower(v2);
            robot.rightOmni.setPower(v2);
        }

        telemetry.addData("dump servo: ", dumpx);
        telemetry.addData("lift count inches: ", liftCount);

    }

    //stolen from the autonomous to automatically align with the white line

    public void encoderDrive(double speed,
                             double leftInches,
                             double timeoutS) {
        int newLeftTarget;

        // Ensure that the opmode is still active

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.rackAndPinion.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            robot.rackAndPinion.setTargetPosition(newLeftTarget);

            // Turn On RUN_TO_POSITION
            robot.rackAndPinion.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.rackAndPinion.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((runtime.seconds() < timeoutS) &&
                    (robot.rackAndPinion.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to  :", newLeftTarget);
                telemetry.addData("Path2",  "Running at  :",
                        robot.rackAndPinion.getCurrentPosition());
                telemetry.update();
            }

            // Turn off RUN_TO_POSITION
//            robot.rackAndPinion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  IF THIS IS ENABLED THE RACK WILL IMMEDIATELY GO BACK DOWN

            //  sleep(250);   // optional pause after each move

    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
