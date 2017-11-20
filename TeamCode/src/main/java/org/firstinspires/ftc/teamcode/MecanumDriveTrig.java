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
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MecanumDriveTrig", group="Pushbot")
public class MecanumDriveTrig extends OpMode {

    /* Declare OpMode members. */
    HardwarePushbotMecanum robot       = new HardwarePushbotMecanum();   // Use a Pushbot's hardware
    public ElapsedTime runtime = new ElapsedTime();


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

//        double forwards;
//        double backwards;
//        double left;
//        double right;
//        // Whole robot moves foreward when left joystick moves up on the y-axis (note: The joystick goes negative when pushed forwards, so negate it)
//        forwards = -gamepad1.left_stick_y; // 1
//        robot.leftFrontMotor.setPower(forwards);
//        robot.leftBackMotor.setPower(forwards);
//        robot.rightFrontMotor.setPower(forwards);
//        robot.rightBackMotor.setPower(forwards);
//
//        backwards = gamepad1.left_stick_y; // -1
//        robot.leftFrontMotor.setPower(backwards);
//        robot.leftBackMotor.setPower(backwards);
//        robot.rightFrontMotor.setPower(backwards);
//        robot.rightBackMotor.setPower(backwards);
//
//        left = gamepad1.left_stick_x; // 1?
//        robot.leftFrontMotor.setPower(-left);
//        robot.leftBackMotor.setPower(left);
//        robot.rightFrontMotor.setPower(left);
//        robot.rightBackMotor.setPower(-left);
//
//        right = -gamepad1.left_stick_x;
//        robot.leftFrontMotor.setPower(right);
//        robot.leftBackMotor.setPower(-right);
//        robot.rightFrontMotor.setPower(-right);
//        robot.rightBackMotor.setPower(right);
//        telemetry.update();
//
//        double turn = gamepad1.right_stick_x; // right stick
//        robot.leftFrontMotor.setPower(turn);
//        robot.leftBackMotor.setPower(turn);
//        robot.rightBackMotor.setPower(-turn);
//        robot.rightBackMotor.setPower(-turn);
        //Thank you reddit
          double r = Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x);
          double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
          double rightX = -gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;
            robot.leftFrontMotor.setPower(v1);
            robot.rightFrontMotor.setPower(v2);
            robot.leftBackMotor.setPower(v3);
            robot.rightBackMotor.setPower(v4);

        telemetry.addData("v1: ",v1);
        telemetry.addData("v2: ",v2);
        telemetry.addData("v3: ",v3);
        telemetry.addData("v4: ",v4);


    }

    //stolen from the autonomous to automatically align with the white line



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
