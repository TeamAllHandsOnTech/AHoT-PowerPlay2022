package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

//@Disabled
@Autonomous(name="BBHazardAltColor", group="B")
public class BBTraumaAltColor extends DriveDirections
{
    OpenCvWebcam webcam;
    ColorSensor frontColor;
    ColorSensor backColor;

    public DistanceSensor distance1 = null;
    public DistanceSensor distance2 = null;
    protected int zone;
    int finalZone;

    private ElapsedTime runtime = new ElapsedTime();
    private double moveSpeed = 0.6;
    private double moveSpeed2 = 0.2;

    public String pickColor(ColorSensor sensor, double sensitivity) {
        if (sensor.red()*1.5>sensitivity*(sensor.green()+sensor.blue())) {return "Red";}
        else if (sensor.green()>sensitivity*(sensor.blue()+sensor.red())) {return "Green";}
        else if (sensor.blue()>sensitivity*(sensor.red()+sensor.green())) {return "Blue";}
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

        distance1 = hardwareMap.get(DistanceSensor.class, "Distance1");
        distance2 = hardwareMap.get(DistanceSensor.class, "Distance2");

        isHazard = false;
        super.runOpMode();
        initArm();

        waitForStart();
        claw.setPosition(0.67);

        runtime.reset();

        straightDrive(moveSpeed, 0.1, "FORWARD");

        rotateToZLoc(-90);

        straightDrive(moveSpeed, 0.6, "FORWARD");

        driveInDirection(moveSpeed, "ROTATE_LEFT");
        sleep(30);
        straightDrive(moveSpeed, 1, "LEFT");
        straightDrive(moveSpeed, 0.1, "BACKWARD");
        driveInDirection(moveSpeed2, "ROTATE_LEFT");
        sleep(80);


        for(int i=5; i>3;i--) {

            boolean frontColorColor = pickColor(frontColor,1.1) == "Red" || pickColor(frontColor,1.1) == "Blue";
            double dist1 = distance1.getDistance(DistanceUnit.MM);
            double dist2 = distance2.getDistance(DistanceUnit.MM);
            double thresA = 2;
            double errorA = (90/Math.PI)*Math.atan((dist1-dist2)/85);

            while (!frontColorColor || Math.abs(errorA)>thresA) {
                frontColorColor = pickColor(frontColor,0.9) == "Red" || pickColor(frontColor,0.9) == "Blue";

                dist1 = distance1.getDistance(DistanceUnit.MM);
                dist2 = distance2.getDistance(DistanceUnit.MM);
                errorA = (90/Math.PI)*Math.atan((dist1-dist2)/85);

                telemetry.addLine("Front color: "+pickColor(frontColor,0.9));
                telemetry.addLine("Angle relative to wall: "+errorA);
                telemetry.addData("distance 1", distance1.getDistance(DistanceUnit.MM));
                telemetry.addData("distance 2", distance2.getDistance(DistanceUnit.MM));
                telemetry.update();

                if (!frontColorColor) {
                    driveInDirection(moveSpeed2, "LEFT");
                } else if (Math.abs(errorA)>thresA) {
                    driveInDirection(Math.signum(errorA)*moveSpeed2/2, "ROTATE_RIGHT");
                }
            }
            driveInDirection(0,"STOP");
            armMotor.setPower(0.8);
            sleep(50*i);
            armMotor.setPower(0.1);
            double errorD = ((dist1+dist2)/2)-100;
            while (Math.abs(errorD)>10) {
                dist1 = distance1.getDistance(DistanceUnit.MM);
                dist2 = distance2.getDistance(DistanceUnit.MM);
                errorD = ((dist1+dist2)/2)-100;
                driveInDirection(Math.signum(errorD)*moveSpeed2/2,"FORWARD");
            }

            driveInDirection(moveSpeed2, "ROTATE_RIGHT");
            sleep(70);
            driveInDirection(0,"STOP");
            straightDrive(moveSpeed, 0.02, "RIGHT");

            claw.setPosition(0.95);
            sleep(500);
            armMotor.setPower(0.8);
            sleep(1200);
            armMotor.setPower(0.1);

            straightDrive(moveSpeed, 0.87, "BACKWARD");
            rotateToZAbs(180, 0);
            straightDrive(moveSpeed, 0.05, "FORWARD");
            sleep(500);
            armMotor.setPower(-0.1);
            sleep(200);
            claw.setPosition(0.67);
            sleep(400);
            armMotor.setPower(0.0);

            straightDrive(moveSpeed, 0.1, "BACKWARD");
            rotateToZAbs(-90, 0);

            straightDrive(moveSpeed, 0.8, "FORWARD");
            straightDrive(moveSpeed, 0.2, "RIGHT");
        }

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