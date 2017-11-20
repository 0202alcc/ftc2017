package org.firstinspires.ftc.teamcode.autonomous;

//Focal length (camera) = (Pixels of object x  Distance (23.5 in)) / Width of known object https://www.pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/
//Uses similar triangles
//// TODO: 11/20/17 Make a manager 
import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class DistanceFromCameraToObject extends OpenCVPipeline {
    double KNOWN_WIDTH = 23.5;
    /*double PIXELS;
    double FOCAL_LENGTH;
    double DISTANCE;*/ //what we are finding

    private Mat frame = new Mat();
    private Mat gray = new Mat();
    private Mat blur = new Mat(g);
    private Mat edged = new Mat();
    private boolean enabled = false;
    public void setEnable(){
        enabled = true;
    }
    public void glyphWall(){
        Imgproc.cvtColor(frame, gray, Imgproc.COLOR_RGB2HSV, 3);
        Imgproc.GaussianBlur(gray, blur, (5,5), 0);
        Imgproc.Canny(blur, edged, 35, 125);
        //Find corners // TODO: 11/20/17
        
    }
    public double findDistance(double focallength, double pixelwidth){
        return (KNOWN_WIDTH * focallength) / pixelwidth;

    }
}
