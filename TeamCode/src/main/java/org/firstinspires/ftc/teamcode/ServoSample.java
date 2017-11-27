package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by gescalona on 11/27/17.
 */

@TeleOp(name="ServoSample", group="Pushbot")
public class ServoSample extends OpMode {
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
        telemetry.addData("Say", "Servo OP mode TEST");    //
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
        if(gamepad1.y){
            robot.lifter.setPosition(1);
        }else if (gamepad1.x){
            robot.lifter.setPosition(0);
        }else if (gamepad1.a) {
            robot.claw.setPosition(0);
        }else if (gamepad1.b){
            robot.claw.setPosition(1);
        }
        telemetry.addData("Servo Lifter: ", robot.lifter.getPosition());
        telemetry.addData("Servo Claw ", robot.claw.getPosition());

    }

    //stolen from the autonomous to automatically align with the white line



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}
