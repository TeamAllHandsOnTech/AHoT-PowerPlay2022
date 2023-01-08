package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="RedCornerHazard", group="A")
public class RedCornerHazard extends DriveDirections
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

        isHazard = true;

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

        StraightDrive(moveSpeed, 0.87, "RIGHT");
        DriveForTime("BACKWARD", moveSpeed, 0.2);
        StraightDrive(0.3, 0.63, "FORWARD");

        armToHeight(925);
        armMotor.setPower(0.1);

        StraightDriveNoSlow(0.22, 0.12, "FORWARD");


        sleep(500);

        openClaw();

        sleep(1000);

        armMotor.setPower(0);

        StraightDrive(moveSpeed, 0.05, "BACKWARD");
        StraightDrive(moveSpeed, 0.36, "LEFT");
        StraightDrive(moveSpeed, 0.47, "FORWARD");

        rotateToZAbs(90, 0);

        StraightDrive(moveSpeed, 1.16, "FORWARD");

        armMotor.setPower(-0.1);

        sleep(300);

        armMotor.setPower(0.1);

        closeClaw();

        sleep(500);

        armToHeight(500);
        armMotor.setPower(0.1);

        sleep(500);

        StraightDrive(moveSpeed, 1.09, "BACKWARD");

        armToHeight(100);

        sleep(500);

        rotateToZAbs(180, 0);

        StraightDrive(moveSpeed, .54, "FORWARD");

        StraightDrive(moveSpeed, .36, "RIGHT");

        armToHeight(550);
        armMotor.setPower(0.1);

        sleep(500);

        StraightDrive(moveSpeed, .07, "FORWARD");

        armMotor.setPower(-0.2);

        sleep(300);

        armMotor.setPower(0);

        openClaw();

        sleep(500);


        telemetry.addData("Final Zone: ", finalZone);
        telemetry.addData("Zone: ", zone);
        telemetry.update();

        switch(finalZone){
            case 1:
                StraightDrive(moveSpeed,1, "RIGHT");
                break;
            case 2:
                StraightDrive(moveSpeed, .3, "RIGHT");
                break;
            case 3:
                StraightDrive(moveSpeed, .3, "LEFT");
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