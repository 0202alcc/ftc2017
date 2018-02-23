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

import org.firstinspires.ftc.teamcode.map.HardwareWombatTwo;

@TeleOp(name="WombatTwo", group="Pushbot")
//@Disabled
public class WombatTwo extends OpMode {

    /* Declare OpMode members. */
    HardwareWombatTwo robot       = new HardwareWombatTwo();   // Use a Pushbot's hardware
    public ElapsedTime runtime = new ElapsedTime();
    boolean foo = false;
    double dumpx;
    double dump2x;
    double intakeValueLeft;
    double intakeValueRight;
    double wombatBat;
    int liftCount = 0;

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

        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        robot.dump.setPosition(0.33);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if(gamepad2.x){
            robot.juul.setPosition(0);
        } else if(gamepad2.b){
            robot.juul.setPosition(1);
        }

        //gamepad2 uses the analog sticks to determine the power of the intake wheels
        intakeValueLeft = gamepad2.left_stick_y;
        intakeValueRight = -gamepad2.right_stick_y;
        robot.leftIntake.setPower(intakeValueLeft);
        robot.rightIntake.setPower(intakeValueRight);

        // left bumper lowers lift, right bumper lifts lift, right triger controls servo
        if (gamepad2.a) {
            robot.rackAndPinion.setPower(1);
        } else if (gamepad2.y){
            robot.rackAndPinion.setPower(-1);
        } else{
            robot.rackAndPinion.setPower(0);
        }

        dumpx = gamepad1.right_trigger;
        dump2x = gamepad2.right_trigger;
        if((dumpx < 1.0 && dumpx > 0) || (dump2x > 0)){
            robot.dump.setPosition(0.5); //Confirm Servo Position
        } else if(dumpx == 1.0){
            robot.dump.setPosition(0.95); //Confirm Servo Position
        } else {
            robot.dump.setPosition(0);
        }

        wombatBat = gamepad2.left_trigger;
        if(wombatBat > 0 && wombatBat < 1){
            robot.bat.setPosition(0.9);
        } else if(wombatBat == 1){
            robot.bat.setPosition(0);
        } else {
            robot.bat.setPosition(0.8);
        }

        /*
         * This is the Mecanum Drive part. The math is explained in the engineering notebook.
         */
        //Thank you reddit user u/2treecko
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
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



        telemetry.addData("dump servo: ", dumpx);
        telemetry.addData("lift count inches: ", liftCount);

    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}