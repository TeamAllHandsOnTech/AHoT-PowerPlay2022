package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.DriveDirections;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name="BlueCornerHazardAltColor", group="B")
public class BlueCornerHazardAltColor extends DriveDirections
{
    OpenCvWebcam webcam;
    ColorSensor frontColor;
    ColorSensor backColor;
    protected int zone;
    int finalZone;

    private ElapsedTime runtime = new ElapsedTime();
    private double moveSpeed = 0.6;
    private double moveSpeed2 = 0.3;

    public String pickColor(ColorSensor sensor) {
        if (sensor.red()>1.3*(sensor.green()+sensor.blue())) {return "Red";}
        else if (sensor.green()>1.3*(sensor.blue()+sensor.red())) {return "Red";}
        else if (sensor.blue()>1.3*(sensor.red()+sensor.green())) {return "Red";}
        else {return "Grey";}
    }

    @Override
    public void runOpMode() {

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        webcam.setPipeline(new SamplePipeline());

        webcam.setMillisecondsPermissionTimeout(1000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        frontColor = hardwareMap.get(ColorSensor.class, "frontColor");
        backColor = hardwareMap.get(ColorSensor.class, "backColor");
        boolean frontColorColor = pickColor(frontColor) == "Red" || pickColor(frontColor) == "Blue";
        boolean backColorColor = pickColor(backColor) == "Red" || pickColor(backColor) == "Blue";


        isHazard = true;

        super.runOpMode();

        //initArm();

        waitForStart();
        //openClaw();

        runtime.reset();

        StraightDrive(moveSpeed, 0.1, "FORWARD");

        rotateToZLoc(-90);

        StraightDrive(moveSpeed, 0.6, "FORWARD");

        DriveInDirection(moveSpeed, "ROTATE_LEFT");
        sleep(25);
        StraightDrive(moveSpeed, 1, "LEFT");
        //for(int i=5; i>3;i--) {
            while (!frontColorColor && !backColorColor) {
                frontColorColor = pickColor(frontColor) == "Red" || pickColor(frontColor) == "Blue";
                backColorColor = pickColor(backColor) == "Red" || pickColor(backColor) == "Blue";

                telemetry.addLine("Front color: "+pickColor(frontColor));
                telemetry.addLine("Back color: "+pickColor(backColor));
                telemetry.update();

                if (!frontColorColor && !backColorColor) {
                    rightFrontDrive.setPower(moveSpeed2/2.4);
                    leftFrontDrive.setPower(-moveSpeed2/2);
                    rightBackDrive.setPower(-moveSpeed2/2);
                    leftBackDrive.setPower(moveSpeed2/2.4);
                } else if (frontColorColor && !backColorColor) {
                    DriveInDirection(moveSpeed2/2, "ROTATE_RIGHT");
                } else if (!frontColorColor && backColorColor) {
                    DriveInDirection(moveSpeed2/2, "ROTATE_LEFT");
                }
            }
            DriveInDirection(0, "STOP");

            /*
            StraightDrive(moveSpeed,0.05,"RIGHT");
            StraightDrive(moveSpeed,0.05,"FORWARD");

            //armToHeight(i*25);
            //closeClaw();

            rightFrontDrive.setPower(-moveSpeed);
            leftFrontDrive.setPower(moveSpeed);
            sleep(200);
            DriveInDirection(0, "STOP");

            StraightDrive(moveSpeed,0.1,"FORWARD");

        }
             */




    }


    class SamplePipeline extends OpenCvPipeline{
        Mat HSVimage = new Mat();
        Mat greenImage = new Mat();
        Mat blueImage = new Mat();
        Mat pinkImage = new Mat();

        double greenPixels;
        double bluePixels;
        double pinkPixels;

        boolean viewportPaused;

        Scalar greenLower = new Scalar(30, 127, 50);
        Scalar greenHigher = new Scalar(90, 255, 175);

        Scalar blueLower = new Scalar(90, 127, 100);
        Scalar blueHigher = new Scalar(100, 255, 255);

        Scalar pinkLower = new Scalar(150, 127, 100);
        Scalar pinkHigher = new Scalar(170, 255, 255);

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, HSVimage, Imgproc.COLOR_RGB2HSV);

            Core.inRange(HSVimage, greenLower, greenHigher, greenImage);
            Core.inRange(HSVimage, blueLower, blueHigher, blueImage);
            Core.inRange(HSVimage, pinkLower, pinkHigher, pinkImage);

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
}