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

package org.firstinspires.ftc.teamcode.map;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareTest {
    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareTest(){
    }

    /* Public OpMode members. */
    public DcMotor leftFrontMotor   = null;
    public DcMotor leftBackMotor  = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor rightBackMotor = null;
    public DcMotor leftOmni = null;
    public DcMotor rightOmni = null;

    public DcMotor rackAndPinion = null;

    public Servo leftIntakeServo = null;
    public Servo rightIntakeServo = null;

    public Servo leftContinuous = null;
    public Servo rightContinuous = null;

    public Servo dump = null;
    public Servo juul = null;
    public Servo bat = null;

    public ColorSensor colorSensor = null;
    public DistanceSensor distanceSensor = null;
    //    public Servo colorservo = null;
//    public ModernRoboticsI2cGyro gyro = null;
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontMotor = hwMap.dcMotor.get("leftFront");
        leftBackMotor = hwMap.dcMotor.get("leftBack");
        rightFrontMotor = hwMap.dcMotor.get("rightFront");
        rightBackMotor = hwMap.dcMotor.get("rightBack");

        leftOmni = hwMap.dcMotor.get("leftOmni");
        rightOmni = hwMap.dcMotor.get("rightOmni");

        rackAndPinion = hwMap.dcMotor.get("rackAndPinion");

        // Define and Initialize Servos

        leftContinuous = hwMap.servo.get("leftContinuous");
        rightContinuous = hwMap.servo.get("rightContinuous");
        juul = hwMap.servo.get("juul");
//        colorservo = hwMap.servo.get("colorservo");
//        colorSensor = hwMap.colorSensor.get("color");
//        gyro = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro");

        dump = hwMap.servo.get("dump");

        bat = hwMap.servo.get("bat");
        colorSensor = hwMap.get(ColorSensor.class, "sensor_color_distance");
        distanceSensor = hwMap.get(DistanceSensor.class, "sensor_color_distance");
//        colorServo = hwMap.servo.get("colorServo");

        // Define and Initialize sensors
//        gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");
//        color = hwMap.colorSensor.get("color");

        // Set Motor Direction
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        leftOmni.setDirection(DcMotor.Direction.REVERSE); //CHECK DIRECTION
        rightOmni.setDirection(DcMotor.Direction.FORWARD); //CHECK DIRECTION

        rackAndPinion.setDirection(DcMotor.Direction.FORWARD); //CHECK DIRECTION
        //Set all motor powers to zero
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

        leftOmni.setPower(0);
        rightOmni.setPower(0);

        rackAndPinion.setPower(0);
        //Establish whether or not you're using encoders
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftOmni.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOmni.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rackAndPinion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */


    public void waitForTick(long periodMs) throws InterruptedException {
        long  remaining = periodMs - (long)period.milliseconds();
        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);
        // Reset the cycle clock for the next pass.
        period.reset();
    }
 }

