package org.firstinspires.ftc.teamcode.map;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by acandidato on 1/10/18.
 */

public class HardwareWombatTwo {
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor <--- whatever that means */
    public HardwareWombatTwo(){
    }

    /* Declare motors and servos and sensors
     */

    //Drive Train
    public DcMotor leftFrontMotor = null;
    public DcMotor leftBackMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor rightBackMotor = null;

    //Intake
    public DcMotor leftIntake = null;
    public DcMotor rightIntake = null;

    //Lift
    public DcMotor rackAndPinion = null;
    public Servo dump = null;

    //Juul Arm
    public Servo juul = null;

    //Bat
    public Servo bat = null;

    //Sensors
    public ColorSensor colorSensor = null;
    public DistanceSensor distanceSensor = null;

    public void init(HardwareMap ahwMap){
        //For referencing
        hwMap = ahwMap;

        /* Call the motors, servos, and sensors to be recognized
         */
        //Drive Train
        leftFrontMotor = hwMap.dcMotor.get("leftFront");
        leftBackMotor = hwMap.dcMotor.get("leftBack");
        rightFrontMotor = hwMap.dcMotor.get("rightFront");
        rightBackMotor = hwMap.dcMotor.get("rightBack");

        //Intake
        leftIntake = hwMap.dcMotor.get("leftIntake");
        rightIntake = hwMap.dcMotor.get("rightIntake");

        //Lift
        rackAndPinion = hwMap.dcMotor.get("rackAndPinion");
        dump = hwMap.servo.get("dump");

        //Juul Arm
        juul = hwMap.servo.get("juul");

        //Bat
        bat = hwMap.servo.get("bat");

        colorSensor = hwMap.get(ColorSensor.class, "sensor_color_distance");
        distanceSensor = hwMap.get(DistanceSensor.class, "sensor_color_distance");
        /* Set Motor Direction
         */
        //Drive
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        //Intake
        leftIntake.setDirection(DcMotor.Direction.FORWARD); //CHECK
        rightIntake.setDirection(DcMotor.Direction.FORWARD); //CHECK

        //Lift
        rackAndPinion.setDirection(DcMotor.Direction.REVERSE);

        /* Set Motor Power to zero
         */
        //Drive
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

        //Intake
        leftIntake.setPower(0);
        rightIntake.setPower(0);

        //Lift
        rackAndPinion.setPower(0);

        /* Set Encoder Status
         */
        //Drive
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Lift
        rackAndPinion.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //These can be overwritten depending on the class thats using it
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
