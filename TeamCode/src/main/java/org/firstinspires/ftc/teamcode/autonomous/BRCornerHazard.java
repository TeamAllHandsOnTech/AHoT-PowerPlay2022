package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Disabled
@Autonomous(name="BRCornerHazard", group="A")
public class BRCornerHazard extends DriveDirections
{
    OpenCvWebcam webcam;
    protected int zone;
    int finalZone;

    private ElapsedTime runtime = new ElapsedTime();
    private double moveSpeed = 0.6;

    @Override
    public void runOpMode(){

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        webcam.setPipeline(new SamplePipeline());

        webcam.setMillisecondsPermissionTimeout(1000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode){

            }
        });

        super.runOpMode();

        initArm();

        waitForStart();

        finalZone = zone;

        telemetry.addData("Final Zone: ", finalZone);
        telemetry.addData("Zone: ", zone);
        telemetry.addData("arm height: ", getArmHeight());
        telemetry.update();

        closeClaw();

        runtime.reset();
        armToHeight(100);
        armMotor.setPower(0.1);

        straightDrive(moveSpeed, 0.87, "RIGHT");
        driveForTime("BACKWARD", moveSpeed, 0.2);
        straightDrive(0.3, 0.63, "FORWARD");

        armToHeight(925);
        armMotor.setPower(0.1);

        straightDriveNoSlow(0.22, 0.12, "FORWARD");


        sleep(500);

        openClaw();

        sleep(1000);

        armMotor.setPower(0);

        straightDrive(moveSpeed, 0.05, "BACKWARD");
        straightDrive(moveSpeed, 0.36, "LEFT");
        straightDrive(moveSpeed, 0.5, "FORWARD");

        rotateToZAbs(90, 0);

        straightDrive(moveSpeed, 1.21, "FORWARD");

//        armMotor.setPower(-0.1);
//
//        sleep(300);

        armMotor.setPower(0.1);

        closeClaw();

        sleep(500);

        armToHeight(500);
        armMotor.setPower(0.1);

        sleep(500);

        straightDrive(moveSpeed, 1.09, "BACKWARD");

        armToHeight(100);

        sleep(500);

        rotateToZAbs(180, 0);

        straightDrive(moveSpeed, .5, "FORWARD");

        straightDrive(moveSpeed, .33, "RIGHT");

        armToHeight(550);
        armMotor.setPower(0.1);

        sleep(500);

        straightDrive(moveSpeed, .10, "FORWARD");

        armMotor.setPower(-0.2);

        sleep(300);

        armMotor.setPower(0);

        openClaw();

        sleep(500);

        straightDrive(moveSpeed,.10, "BACKWARD");


        telemetry.addData("Final Zone: ", finalZone);
        telemetry.addData("Zone: ", zone);
        telemetry.update();

        switch(finalZone){
            case 1:
                straightDrive(moveSpeed,1, "RIGHT");
                break;
            case 2:
                straightDrive(moveSpeed, .3, "RIGHT");
                break;
            case 3:
                straightDrive(moveSpeed, .4, "LEFT");
                break;
        }

        while (opModeIsActive()){

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

        Scalar greenLower = new Scalar(30, 75, 75);
        Scalar greenHigher = new Scalar(90, 255, 255);

        Scalar blueLower = new Scalar(90, 110, 50);
        Scalar blueHigher = new Scalar(100, 255, 255);

        Scalar pinkLower = new Scalar(150, 100, 100);
        Scalar pinkHigher = new Scalar(170, 200, 255);


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