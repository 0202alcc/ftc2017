package org.firstinspires.ftc.teamcode.autonomous;

//Focal length (camera) = (Pixels of object x  Distance (23.5 in)) / Width of known object https://www.pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/
//Uses similar triangles
//// TODO: 11/20/17 Make a manager 
import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Scalar;

import java.util.ArrayList;

public class DistanceFromCameraToObject extends OpenCVPipeline {
    private double KNOWN_WIDTH = 23.5;
    //private double KNOWN_LENGTH = 24;
    private double FOCAL_LENGTH;
    private double DISTANCE = 15;// what we are dfinding

    private Mat agray = new Mat();
    private Mat blur = new Mat();
    private Mat edged = new Mat();
    private ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
    private boolean enabled = false;
    public void setEnable(boolean boo){
        enabled = boo;
    }
    public double findDistance(double focallength, double pixelwidth){
        return (KNOWN_WIDTH * focallength) / pixelwidth;

    }
    @Override
    public Mat processFrame(Mat rgba, Mat gray){
        Imgproc.cvtColor(rgba, agray, Imgproc.COLOR_RGB2HSV_FULL);
        Imgproc.GaussianBlur(agray, blur, new Size(5,5), 0);
        Imgproc.Canny(blur, edged, 35, 125);
        Imgproc.findContours(edged, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        for(int i = 0; i < contours.size(); i++){
            Imgproc.drawContours(edged, contours, i, new Scalar(255, 255, 255), -1);
        }
        if (enabled){
            //DISTANCE = findDistance();
            return edged;
        }

        return rgba;

    }

}
