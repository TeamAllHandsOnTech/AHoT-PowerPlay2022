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
@Autonomous(name="BlueCornerHazardVision", group="Hazard")
    public class BlueCornerHazardVision extends DriveDirections
    {
        OpenCvWebcam webcam;
        protected int zone;
        int finalZone;

        private ElapsedTime runtime = new ElapsedTime();
        private double moveSpeed = 0.3;


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
                telemetry.update();

                closeClaw();

                runtime.reset();

                DriveForDistance(moveSpeed, .50, "RIGHT");
                DriveForDistance(moveSpeed, .07, "BACKWARD");
                DriveForDistance(moveSpeed, .12, "FORWARD");
                openClaw();

                sleep(3000);

                    DriveForTime("BACKWARD", moveSpeed, 1);

                    telemetry.addData("Final Zone: ", finalZone);
                    telemetry.addData("Zone: ", zone);
                    telemetry.update();

                    switch(finalZone){
                        case 1:
                            DriveForDistance(moveSpeed, 1.1, "LEFT");
                            DriveForDistance(moveSpeed, .2, "BACKWARD");
                            DriveForDistance(moveSpeed, .75, "FORWARD");
                            break;
                        case 2:
                            DriveForDistance(moveSpeed, .36, "LEFT");
                            DriveForDistance(moveSpeed, .05, "BACKWARD");
                            DriveForDistance(moveSpeed, .75, "FORWARD");
                            break;
                        case 3:
                            DriveForDistance(moveSpeed, .36, "RIGHT");
                            DriveForDistance(moveSpeed, .05, "BACKWARD");
                            DriveForDistance(moveSpeed, .75, "FORWARD");
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

                    Scalar greenLower = new Scalar(30, 127, 20);
                    Scalar greenHigher = new Scalar(90, 200, 155);

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