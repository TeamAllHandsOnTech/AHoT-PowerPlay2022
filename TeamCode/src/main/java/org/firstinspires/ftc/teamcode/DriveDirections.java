package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class DriveDirections extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor rightFrontDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftBackDrive = null;
    private double moveSpeed = 0.3;

    private static double ARM_MIN_RANGE = 0.575;
    private static double ARM_MAX_RANGE = 0.35;

    public DcMotor armMotor = null;

    public Servo claw = null;

    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;

    double previousHeading = 0;
    double intergratedHeading = 0;



    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        //Calibrate NavX
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope)navxMicro;
        telemetry.addLine("Gyro Calibrating. Do Not Move!");
        telemetry.update();

        // Wait until the gyro calibration is complete
        while (navxMicro.isCalibrating())  {
            telemetry.addLine("calibrating...");
            telemetry.update();
            sleep(50);
        }
        telemetry.addLine("Gyro Calibrated. Press Start.");
        telemetry.update();

    }

    public void DriveInDirection(double power, String direction){
        switch(direction){
            case "FORWARD":
                rightFrontDrive.setPower(power);
                leftFrontDrive.setPower(power);
                rightBackDrive.setPower(power);
                leftBackDrive.setPower(power);
                break;

            case "BACKWARD":
                rightFrontDrive.setPower(-power);
                leftFrontDrive.setPower(-power);
                rightBackDrive.setPower(-power);
                leftBackDrive.setPower(-power);
                break;

            case "LEFT":
                rightFrontDrive.setPower(power);
                leftFrontDrive.setPower(-power);
                rightBackDrive.setPower(-power);
                leftBackDrive.setPower(power);
                break;

            case "RIGHT":
                rightFrontDrive.setPower(-power);
                leftFrontDrive.setPower(power);
                rightBackDrive.setPower(power);
                leftBackDrive.setPower(-power);
                break;
            case "ROTATE_RIGHT":
                rightFrontDrive.setPower(-power);
                leftFrontDrive.setPower(power);
                rightBackDrive.setPower(-power);
                leftBackDrive.setPower(power);
                break;
            case "ROTATE_LEFT":
                rightFrontDrive.setPower(power);
                leftFrontDrive.setPower(-power);
                rightBackDrive.setPower(power);
                leftBackDrive.setPower(-power);
                break;
            case "STOP":
                rightFrontDrive.setPower(0);
                leftFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftBackDrive.setPower(0);
                break;
        }
    }

    public void DriveForDistance (double power, double distance, String direction){
        double clicksPerMeter = 2492.788;
        double targetClicks = distance*clicksPerMeter;
        double currentClicks = 0;

        double rightFrontClicks = 0;
        double leftFrontClicks = 0;
        double rightBackClicks = 0;
        double leftBackClicks = 0;

        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DriveInDirection(power, direction);

        while(currentClicks < targetClicks){

            rightFrontClicks = Math.abs(rightFrontDrive.getCurrentPosition());
            leftFrontClicks = Math.abs(leftFrontDrive.getCurrentPosition());
            rightBackClicks = Math.abs(rightBackDrive.getCurrentPosition());
            leftBackClicks = Math.abs(leftBackDrive.getCurrentPosition());

            currentClicks = (rightFrontClicks+leftFrontClicks+rightBackClicks+leftBackClicks)/4;
        }
        DriveInDirection(0, "FORWARD");
    }

    public void DriveForTime(String direction, double power, double time){
        time*=1000;
        DriveInDirection(power, direction);
        sleep((long)time);
        DriveInDirection(power, "FORWARD");
    }

    public void StraightDrive(double power, double dist, String direction, double errorThresh){
        intergratedHeading = 0;

        double startAngle = getCumulativeZ();
        double error = getCumulativeZ() - startAngle;
        double powerDifference = error / 90;
        double clicksPerMeter = 2492.788;
        double targetClicks = dist*clicksPerMeter;
        double currentClicks = 0;

        double rightFrontClicks = 0;
        double leftFrontClicks = 0;
        double rightBackClicks = 0;
        double leftBackClicks = 0;

        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        switch(direction){
            case "FORWARD":
                while(currentClicks < targetClicks) {
                    error = getCumulativeZ() - startAngle;
                    rightFrontClicks = Math.abs(rightFrontDrive.getCurrentPosition());
                    leftFrontClicks = Math.abs(leftFrontDrive.getCurrentPosition());
                    rightBackClicks = Math.abs(rightBackDrive.getCurrentPosition());
                    leftBackClicks = Math.abs(leftBackDrive.getCurrentPosition());

                    currentClicks = (rightFrontClicks + leftFrontClicks + rightBackClicks + leftBackClicks) / 4;
                    powerDifference = Math.abs(error / 90);
                    if (error > errorThresh) {
                        rightFrontDrive.setPower(power);
                        leftFrontDrive.setPower(power + powerDifference);
                        rightBackDrive.setPower(power);
                        leftBackDrive.setPower(power + powerDifference);
                        telemetry.addLine("turning right");
                        telemetry.update();
                    } else if (error < -errorThresh) {
                        rightFrontDrive.setPower(power + powerDifference);
                        leftFrontDrive.setPower(power);
                        rightBackDrive.setPower(power + powerDifference);
                        leftBackDrive.setPower(power);
                        telemetry.addLine("turning left");
                        telemetry.update();
                    } else {
                        telemetry.addLine("straight");
                        telemetry.update();
                        rightFrontDrive.setPower(power);
                        leftFrontDrive.setPower(power);
                        rightBackDrive.setPower(power);
                        leftBackDrive.setPower(power);
                    }
                }
                DriveInDirection(0, "FORWARD");
                break;

            case "BACKWARD":
                while(currentClicks < targetClicks){
                    error = getCumulativeZ() - startAngle;
                    rightFrontClicks = Math.abs(rightFrontDrive.getCurrentPosition());
                    leftFrontClicks = Math.abs(leftFrontDrive.getCurrentPosition());
                    rightBackClicks = Math.abs(rightBackDrive.getCurrentPosition());
                    leftBackClicks = Math.abs(leftBackDrive.getCurrentPosition());

                    currentClicks = (rightFrontClicks+leftFrontClicks+rightBackClicks+leftBackClicks)/4;
                    powerDifference = Math.abs(error / 90);
                    if(error > errorThresh){
                        rightFrontDrive.setPower(-power);
                        leftFrontDrive.setPower(-(power + powerDifference));
                        rightBackDrive.setPower(-power);
                        leftBackDrive.setPower(-(power + powerDifference));
                        telemetry.addLine("turning right");
                        telemetry.addData("error", error);
                        telemetry.update();
                    }else if(error < -errorThresh){
                        rightFrontDrive.setPower(-(power + powerDifference));
                        leftFrontDrive.setPower(-power);
                        rightBackDrive.setPower(-(power + powerDifference));
                        leftBackDrive.setPower(-power);
                        telemetry.addLine("turning left");
                        telemetry.addData("error", error);
                        telemetry.update();
                    }else{
                        telemetry.addLine("straight");
                        telemetry.addData("error", error);
                        telemetry.update();
                        rightFrontDrive.setPower(-power);
                        leftFrontDrive.setPower(-power);
                        rightBackDrive.setPower(-power);
                        leftBackDrive.setPower(-power);
                    }
                }
                DriveInDirection(0, "FORWARD");
                break;

            case "LEFT":
                while(currentClicks < targetClicks) {
                    error = getCumulativeZ() - startAngle;
                    rightFrontClicks = Math.abs(rightFrontDrive.getCurrentPosition());
                    leftFrontClicks = Math.abs(leftFrontDrive.getCurrentPosition());
                    rightBackClicks = Math.abs(rightBackDrive.getCurrentPosition());
                    leftBackClicks = Math.abs(leftBackDrive.getCurrentPosition());

                    currentClicks = (rightFrontClicks + leftFrontClicks + rightBackClicks + leftBackClicks) / 4;
                    powerDifference = Math.abs(error / 90);
                    if (error > errorThresh) {
                        rightFrontDrive.setPower(power + powerDifference);
                        leftFrontDrive.setPower(-(power + powerDifference));
                        rightBackDrive.setPower(-(power));
                        leftBackDrive.setPower(power);
                        telemetry.addLine("turning right");
                        telemetry.update();
                    } else if (error < -errorThresh) {
                        rightFrontDrive.setPower(power);
                        leftFrontDrive.setPower(-power);
                        rightBackDrive.setPower(-(power + powerDifference));
                        leftBackDrive.setPower(power + powerDifference);
                        telemetry.addLine("turning left");
                        telemetry.update();
                    } else {
                        telemetry.addLine("straight");
                        telemetry.update();
                        rightFrontDrive.setPower(power);
                        leftFrontDrive.setPower(-power);
                        rightBackDrive.setPower(-power);
                        leftBackDrive.setPower(power);
                    }
                }
                DriveInDirection(0, "FORWARD");
                break;

            case "RIGHT":
                while(currentClicks < targetClicks) {
                    error = getCumulativeZ() - startAngle;
                    rightFrontClicks = Math.abs(rightFrontDrive.getCurrentPosition());
                    leftFrontClicks = Math.abs(leftFrontDrive.getCurrentPosition());
                    rightBackClicks = Math.abs(rightBackDrive.getCurrentPosition());
                    leftBackClicks = Math.abs(leftBackDrive.getCurrentPosition());

                    currentClicks = (rightFrontClicks + leftFrontClicks + rightBackClicks + leftBackClicks) / 4;
                    powerDifference = Math.abs(error / 90);
                    if (error > errorThresh) {
                        rightFrontDrive.setPower(power);
                        leftFrontDrive.setPower(power);
                        rightBackDrive.setPower(power);
                        leftBackDrive.setPower(-power);
                        telemetry.addLine("turning right");
                        telemetry.update();
                    } else if (error < -errorThresh) {
                        rightFrontDrive.setPower(-(power + powerDifference));
                        leftFrontDrive.setPower(power + powerDifference);
                        rightBackDrive.setPower(power);
                        leftBackDrive.setPower(power);
                        telemetry.addLine("turning left");
                        telemetry.update();
                    } else {
                        telemetry.addLine("straight");
                        telemetry.update();
                        rightFrontDrive.setPower(-power);
                        leftFrontDrive.setPower(power);
                        rightBackDrive.setPower(power);
                        leftBackDrive.setPower(-power);
                    }
                }
                DriveInDirection(0, "FORWARD");
                break;
            case "STOP":
                rightFrontDrive.setPower(0);
                leftFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftBackDrive.setPower(0);
                break;
        }

    }

    public void rotateToZAbs(double targetAngle, double power){
        double angle = getCumulativeZ();
        double difference = targetAngle - angle;
        if(difference > 0){
            while (angle < targetAngle) {
                angle = getCumulativeZ();
                DriveInDirection(power,"ROTATE_RIGHT");
            }
        } else if(difference < 0){
            while (angle > targetAngle) {
                angle = getCumulativeZ();
                DriveInDirection(power,"ROTATE_LEFT");
            }
        }

        DriveInDirection(0,"STOP");

    }
    //WIPPPPPPPP!!!!
    public void rotateToZLoc(double localTargetAngle, double dividend){
        intergratedHeading = 0;
        double startAngle = getCumulativeZ();
        double absTargetAngle = localTargetAngle + startAngle;
        double error = absTargetAngle - getCumulativeZ();
        while(Math.abs(error) > 1){
            while (error > 1) {


                error = absTargetAngle - getCumulativeZ();
                //rotate left
                DriveInDirection(error / dividend, "ROTATE_RIGHT");


    //            //telemetry
    //            telemetry.addLine("currentZ: " + getCurrentZ());
                telemetry.addLine("cumulativeZ: " + getCumulativeZ());
                telemetry.addLine("Error: " + error);
                telemetry.addLine("rotation: counter clockwise");
                telemetry.update();
            }

            while (error < -1) {


                error = absTargetAngle - getCumulativeZ();
                //rotate right
                DriveInDirection(error / dividend, "ROTATE_LEFT");

                //telemetry
    //            telemetry.addLine("currentZ" + getCurrentZ());
                telemetry.addLine("cumulativeZ" + getCumulativeZ());
                telemetry.addLine("Error: " + error);
                telemetry.addLine("targetAngle: " + localTargetAngle);
                telemetry.addLine("rotation: clockwise");
                telemetry.update();
            }


        rightFrontDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);

        }
    }


    public int fakeVision() {
        int location;
        location = (int) Math.random() * 3 + 1;
        if (location==1) {
            return 1;
        } else if (location==2) {
            return 2;
        } else if (location==3) {
            return 3;
        }else{
            return 0;
        }
    }


    public double getCurrentZ() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public double getCumulativeZ() {
        double currentHeading = getCurrentZ();
        double deltaHeading = currentHeading - previousHeading;
        if (deltaHeading <= -180) {
            deltaHeading += 360;
        } else if (deltaHeading >= 180) {
            deltaHeading -= 360;
        }

        intergratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return  intergratedHeading;
    }

    //Test code for arm
    //Gets height in millimeters
    public void initArm() {
        double Slow_Arm_Speed = 0.3;
        double Fast_Arm_Speed = 0.55;
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(ARM_MIN_RANGE);
        telemetry.addData("min pos",claw.MIN_POSITION);
        telemetry.addData("max pos", claw.MAX_POSITION);

        telemetry.addData("current height", getArmHeight());
        telemetry.update();
    }

    public boolean isClawOpen() {
        if (claw.getPosition()>0.1) {return false;}
        else { return true;}

    }

    public void closeClaw(){
        claw.setPosition(ARM_MIN_RANGE);

    }

    public void openClaw(){
        claw.setPosition(ARM_MAX_RANGE);

    }

    public double getArmHeight() {
        return armMotor.getCurrentPosition()/3.433;
    }

    public void armStop() {
        armMotor.setPower(0);
    }
    //distance is in millimeters
    public void armToHeight (double power, double targetHeight){

        double currentHeight = getArmHeight();
        double error = targetHeight + currentHeight;

        if (targetHeight>currentHeight) {
            if (Math.abs(error) > 100) {
                telemetry.addData("error: ", error);
                telemetry.addData("Current Height: ", currentHeight);
                telemetry.update();
                armMotor.setPower(-error / 100);
                currentHeight = getArmHeight();
                error = targetHeight + currentHeight;
            }
        } else if (targetHeight<currentHeight) {

            if (Math.abs(error) > 100) {
                armMotor.setPower(error / 854);
                currentHeight = getArmHeight();
                error = targetHeight + currentHeight;
            }
        } else {
            armMotor.setPower(0);
        }
    }
//https://rosettacode.org/wiki/Map_range
    //used this explanation
    public double MapRange(double min1, double max1, double min2, double max2, double input){
        return(min2 + ((input - min1)*(max2 - min2)/(max1 - min1)));
    }



}
