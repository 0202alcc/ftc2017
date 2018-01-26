package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.HardwareWombatTwo;

/**
 * Created by gescalona on 1/12/18.
 */

public class VuforiaEncoder  {
    private HardwareMap robot;
    private VuforiaLocalizer.Parameters parameters;
    public VuforiaEncoder(HardwareMap robot, VuforiaLocalizer.Parameters parameters) {
        this.robot = robot;
        this.parameters = parameters;

    }

    public static final String TAG = "Vuforia VuMark Sample";
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;
    public void enable(){
        parameters.vuforiaLicenseKey = "AWsJ5iL/////AAAAGcydaMOy3UO3nEa/8sl/KSQoZXrNfd5+zhs8IbQdAzh/0alGOmxW0JULoug0Y7kugTpJu5wu3YPAJAcYtSLJFIxZ7t34g9f7XqNeUKXall1mAVbzWh5aG0FNaoflkkKwPtj+eOVjZo4bA6TZ6f9WFChvsdlaao7me/9/qBG06GSYxKR4oIksiGXHV2uo8ROziIN7x1bevU4g7VHPrgQIpygIVPuDW+Bq2blvKOtqcYop3q54k1tfhhPiA2oSLdKgFOuOEIMDf/kLZDQ4F2RQHGLJIRvErcbMwUoGs1eEa01xT21r767Ve72pvU8m/GC/fx2oTizi1z+bEvtKjBu+YeW760OpGhZKCe1ECPjt772z";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
    }
    public void activate(){
        relicTrackables.activate();
    }
    public void track(Telemetry telemetry){
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
            telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;
            }
        } else {
            telemetry.addData("VuMark", "not visible");
        }
            telemetry.update();
        }
}
