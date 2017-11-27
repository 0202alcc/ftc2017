package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.image.ColorVision;

/**
 * Created by guinea on 10/5/17.
 * This is a sample opmode that demonstrates the use of an OpenCVPipeline with FTC code.
 * When the x button is pressed on controller one, the camera is set to show areas of the image
 * where a certain color is, in this case, blue.
 *
 */
@TeleOp(name="DistanceFromCameraToObject")
public class ColorDemo extends OpMode {
    DistanceFromCameraToObject distob = new DistanceFromCameraToObject();
    @Override
    public void init() {
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        distob.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        distob.enable(false);
        // start the vision system
        distob.enable();
    }

    @Override
    public void loop() {
        distob.setEnable(gamepad1.x);
    }

    public void stop() {
        // stop the vision system
        distob.disable();
    }
}
