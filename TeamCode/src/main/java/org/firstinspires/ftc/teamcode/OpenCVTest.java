package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCVTest extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat image) {
        Mat hsvImage = new Mat();
        Mat greenImage = new Mat();
        Mat blueImage = new Mat();
        Mat pinkImage = new Mat();

        double greenPixels;
        double bluePixels;
        double pinkPixels;
        int zone;

        Imgproc.cvtColor(image, hsvImage, Imgproc.COLOR_RGB2HSV);

        //Green
        Scalar greenLower = new Scalar(60, 127, 50);
        Scalar greenHigher = new Scalar(80, 255, 255);

        Core.inRange(hsvImage, greenLower, greenHigher, greenImage);

        //Blue
        Scalar blueLower = new Scalar(80, 127, 100);
        Scalar blueHigher = new Scalar(100, 255, 255);

        Core.inRange(hsvImage, blueLower, blueHigher, blueImage);

        //Pink
        Scalar pinkLower = new Scalar(150, 127, 100);
        Scalar pinkHigher = new Scalar(170, 255, 255);

        Core.inRange(hsvImage, pinkLower, pinkHigher, pinkImage);

        greenPixels = Core.sumElems(greenImage).val[0];
        bluePixels = Core.sumElems(blueImage).val[0];
        pinkPixels = Core.sumElems(pinkImage).val[0];

        if(greenPixels > bluePixels && greenPixels > pinkPixels){
            zone = 1;
        } else if(bluePixels > pinkPixels){
            zone = 2;
        } else {
            zone = 3;
        }

        return null;
    }
}