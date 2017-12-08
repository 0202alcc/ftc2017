package org.firstinspires.ftc.teamcode.autonomous;

//Focal length (camera) = (Pixels of object x  Distance (23.5 in)) / Width of known object https://www.pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/
//Uses similar triangles
//// TODO: 11/20/17 Make a manager
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Scalar;

import java.util.ArrayList;


public class DistanceFromCameraToObject /*extends OpenCVPipeline */ {/*
    private double KNOWN_WIDTH = 23.5;
    //private double KNOWN_LENGTH = 24;
    private double FOCAL_LENGTH;
    private double DISTANCE = 15;// what we are dfinding

    MatOfPoint2f con2f;

    private Mat agray = new Mat();
    private Mat blur = new Mat();
    private Mat edged = new Mat();
    private ArrayList<MatOfPoint> contours = new ArrayList<>();
    private boolean enabled = false;
    private boolean enabled2 = false;
    private double areaa = -567;
    public void setEnable(boolean boo){
        enabled = boo;
    }
    public boolean isEnabled(){
        return enabled;
    }
    public boolean isEnabled2(){
        return
    }
    public double findDistance(double width, double focallength, double pixelwidth){
        return (width * focallength) / pixelwidth;

    }
    public double getArea (){
        return areaa;
    }
    @Override
    public Mat processFrame(Mat rgba, Mat gray){
        if (enabled) {
            Imgproc.cvtColor(rgba, agray, Imgproc.COLOR_RGB2HSV_FULL);
            Imgproc.GaussianBlur(agray, blur, new Size(5, 5), 0);
            Imgproc.Canny(blur, edged, 35, 125);
            Imgproc.findContours(edged, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            double maxArea = 0;
            for (MatOfPoint con : contours) {
                con2f = new MatOfPoint2f(con.toArray());
                double peri = Imgproc.arcLength(con2f, true);
                Imgproc.approxPolyDP(con2f, con2f, (0.02 * peri), true);
                if (Imgproc.contourArea(con) > maxArea) {
                    maxArea = Imgproc.contourArea(con);
                }
                areaa = maxArea;
            }

            Imgproc.drawContours(edged, contours, 2, new Scalar(255, 255, 255), -1);
            //DISTANCE = findDistance();
            if(isEnabled2()){
                //return Imgproc.minAreaRect(con2f);
            }
            return edged;
        }else {
            return rgba;
        }
    }
    */
}

