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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

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

@TeleOp(name="UltroDrive", group="Pushbot")
@Disabled
public class UltroDrive extends OpMode {
    double          intakeOffset  = 0.0 ;                  // Servo mid position
    final double    INTAKE_SPEED  = 0.02 ;
    double          CLOSED = 0.75;

    static final double INCREMENT   = 0.001; // amount to ramp motor each CYCLE_MS cycle
    static final double DECREASE    = -0.01;
    static final double MAX_FWD     =  1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV     =  0;     // Maximum REV power applied to motor

    double  power   = 0;
    boolean rampUp  = true;
    /* Declare OpMode members. */
    org.firstinspires.ftc.teamcode.HardwarePushbot robot       = new org.firstinspires.ftc.teamcode.HardwarePushbot();   // Use a Pushbot's hardware
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
        robot.limiter.setPosition(1);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
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

        double left;
        double right;
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        robot.leftFrontMotor.setPower(left);
        robot.leftBackMotor.setPower(left);
        robot.rightFrontMotor.setPower(right);
        robot.rightBackMotor.setPower(right);
        robot.flyWheelMotor.setPower(power);

        if(gamepad2.dpad_up){
            robot.intakeMotor.setPower(1.0);
        }
        if(gamepad2.dpad_down){
            robot.intakeMotor.setPower(-1.0);
        }
        if(gamepad2.dpad_left) {
            robot.intakeMotor.setPower(0);
        }
        if(gamepad2.dpad_right) {
            robot.intakeMotor.setPower(0);
        }

        if(gamepad2.right_bumper) {
            power += INCREMENT;
            if (power >= MAX_FWD) {
                power = MAX_FWD;
                rampUp = true;
            }
        }
        if (gamepad2.left_bumper) {
            power -= INCREMENT;
            if (power <= 0.0) {
                power = 0.0;
                rampUp = true;
            }
        }
        if(power <= 0.0) {
            power = 0.0;
        }
        if(power >= 1.0){
            power = 1.0;
        }
        if(gamepad2.a){
            robot.limiter.setPosition(.2);
        }
        else {
            robot.limiter.setPosition(1);
        }

        if(gamepad1.x)
            robot.leftBeacon.setPosition(0.16);

        if(gamepad1.y)
            robot.leftBeacon.setPosition(1);

        if(gamepad1.a)
            robot.rightBeacon.setPosition(0.16);

        if(gamepad1.b)
            robot.rightBeacon.setPosition(1);



        telemetry.addData("Flywheel Power", "%5.2f", power);
        telemetry.update();

    }

    //stolen from the autonomous to automatically align with the white line



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
