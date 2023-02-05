package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class DriveDirections extends LinearOpMode {

    public DcMotor rightFrontDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftBackDrive = null;

    //Claw values for Trauma
    private static final double ARM_MIN_RANGE = .65;
    private static final double ARM_MAX_RANGE = 0.4;

    //Claw values for Hazard, uncomment if using Hazard

    //    private static final double ARM_MIN_RANGE = .88;
    //    private static final double ARM_MAX_RANGE = 0.7;

    public DcMotor armMotor = null;

    public Servo claw = null;

    public DistanceSensor distance1 = null;
    public DistanceSensor distance2 = null;

    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;

    double previousHeading = 0;
    double integratedHeading = 0;

    public boolean isHazard;



    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //distance sensors
        if(!isHazard) {
            distance1 = hardwareMap.get(DistanceSensor.class, "Distance1");
            distance2 = hardwareMap.get(DistanceSensor.class, "Distance2");
        }

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

    public void driveInDirection(double power, String direction){
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

    public void driveForDistance(double power, double distance, String direction){
        double clicksPerMeter = 2492.788;
        double targetClicks = distance*clicksPerMeter;
        double currentClicks = 0;

        double rightFrontClicks;
        double leftFrontClicks;
        double rightBackClicks;
        double leftBackClicks;

        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveInDirection(power, direction);

        while(currentClicks < targetClicks){

            rightFrontClicks = Math.abs(rightFrontDrive.getCurrentPosition());
            leftFrontClicks = Math.abs(leftFrontDrive.getCurrentPosition());
            rightBackClicks = Math.abs(rightBackDrive.getCurrentPosition());
            leftBackClicks = Math.abs(leftBackDrive.getCurrentPosition());

            currentClicks = (rightFrontClicks+leftFrontClicks+rightBackClicks+leftBackClicks)/4;
        }
        driveInDirection(0, "FORWARD");
    }

    public void straightDrive(double power, double distance, String direction){
        double clicksPerMeter = 2492.788;
        double targetClicks = distance*clicksPerMeter;
        double currentClicks = 0;

        double RFPower;
        double LFPower;
        double RBPower;
        double LBPower;

        double rightFrontClicks;
        double leftFrontClicks;
        double rightBackClicks;
        double leftBackClicks;

        integratedHeading = 0;

        double target = getCumulativeZ();
        double error = getCumulativeZ() - target;
        double powerDifference = error / 90;

        double slowDownDistance = 0.25 * clicksPerMeter;
        double powerMult;
        double minPowerMult = 0;
        boolean reachedMinimum = false;


        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveInDirection(power, direction);

        RFPower = rightFrontDrive.getPower();
        LFPower = leftFrontDrive.getPower();
        RBPower = rightBackDrive.getPower();
        LBPower = leftBackDrive.getPower();


        while(currentClicks < targetClicks){

            error = getCumulativeZ() - target;
            powerDifference = error / 90;




            rightFrontClicks = Math.abs(rightFrontDrive.getCurrentPosition());
            leftFrontClicks = Math.abs(leftFrontDrive.getCurrentPosition());
            rightBackClicks = Math.abs(rightBackDrive.getCurrentPosition());
            leftBackClicks = Math.abs(leftBackDrive.getCurrentPosition());

            currentClicks = (rightFrontClicks+leftFrontClicks+rightBackClicks+leftBackClicks)/4;

            powerMult = ((targetClicks - currentClicks) / slowDownDistance) + 0.25;
            if(powerMult > 1){
                powerMult = 1;
            }
            if(powerMult < 0.1){
                powerMult = 0.25;
            }

            if(currentClicks < targetClicks - slowDownDistance) {
                rightFrontDrive.setPower(RFPower - powerDifference);
                leftFrontDrive.setPower(LFPower + powerDifference);
                rightBackDrive.setPower(RBPower - powerDifference);
                leftBackDrive.setPower(LBPower + powerDifference);
            }else{
                if(Math.abs(RFPower * powerMult) <= 0.25 && !reachedMinimum){
                    reachedMinimum = true;
                    minPowerMult = powerMult;
                }
                if(!reachedMinimum) {
                    rightFrontDrive.setPower((RFPower * powerMult) - powerDifference);
                    leftFrontDrive.setPower((LFPower * powerMult) + powerDifference);
                    rightBackDrive.setPower((RBPower * powerMult) - powerDifference);
                    leftBackDrive.setPower((LBPower * powerMult) + powerDifference);
                }else{
                    rightFrontDrive.setPower((RFPower * minPowerMult) - powerDifference);
                    leftFrontDrive.setPower((LFPower * minPowerMult) + powerDifference);
                    rightBackDrive.setPower((RBPower * minPowerMult) - powerDifference);
                    leftBackDrive.setPower((LBPower * minPowerMult) + powerDifference);
                }
            }

            telemetry.addData("PowerMult: ", powerMult);
            telemetry.update();

        }
        driveInDirection(0, "FORWARD");
    }
    //no slowdown as robot approaches target
    public void straightDriveNoSlow(double power, double distance, String direction){
        double clicksPerMeter = 2492.788;
        double targetClicks = distance*clicksPerMeter;
        double currentClicks = 0;

        double RFPower;
        double LFPower;
        double RBPower;
        double LBPower;

        double rightFrontClicks;
        double leftFrontClicks;
        double rightBackClicks;
        double leftBackClicks;

        integratedHeading = 0;

        double target = getCumulativeZ();
        double error = getCumulativeZ() - target;
        double powerDifference = error / 90;


        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveInDirection(power, direction);

        RFPower = rightFrontDrive.getPower();
        LFPower = leftFrontDrive.getPower();
        RBPower = rightBackDrive.getPower();
        LBPower = leftBackDrive.getPower();


        while(currentClicks < targetClicks){

            error = getCumulativeZ() - target;
            powerDifference = error / 90;

            rightFrontClicks = Math.abs(rightFrontDrive.getCurrentPosition());
            leftFrontClicks = Math.abs(leftFrontDrive.getCurrentPosition());
            rightBackClicks = Math.abs(rightBackDrive.getCurrentPosition());
            leftBackClicks = Math.abs(leftBackDrive.getCurrentPosition());

            currentClicks = (rightFrontClicks+leftFrontClicks+rightBackClicks+leftBackClicks)/4;

            if(currentClicks < targetClicks) {
                rightFrontDrive.setPower(RFPower - powerDifference);
                leftFrontDrive.setPower(LFPower + powerDifference);
                rightBackDrive.setPower(RBPower - powerDifference);
                leftBackDrive.setPower(LBPower + powerDifference);
            }
        }
        driveInDirection(0, "FORWARD");
    }

    public void driveForTime(String direction, double power, double time){
        time*=1000;
        driveInDirection(power, direction);
        sleep((long)time);
        driveInDirection(power, "FORWARD");
    }

    public void rotateToZAbs(double absTargetAngle, double zero){
        double dividend = 50;
        sleep(20);
        integratedHeading = 0;
        double startAngle = getCurrentZ();
        double error = zero + absTargetAngle - getCurrentZ();

        if(error > 180){
            error -= 360;
        }
        if(error < -180){
            error += 360;
        }

        while(Math.abs(error) > 0.1){
            while (error > 0.1) {
                error = zero + absTargetAngle - getCurrentZ();
                if(error > 180){
                    error -= 360;
                }
                if(error < -180){
                    error += 360;
                }
                double proportionalPower = error / dividend;
                proportionalPower = Math.abs(proportionalPower);

                if(proportionalPower < 0.1){
                    proportionalPower = 0.1;
                }

                //rotate left
                driveInDirection(proportionalPower, "ROTATE_LEFT");


                telemetry.addLine("StartAngle: " + startAngle);
                telemetry.addLine("TargetAngle: " + absTargetAngle);
                telemetry.addLine("CumulativeZ: " + getCurrentZ());
                telemetry.addLine("Error: " + error);
                telemetry.addLine("rotation: counter clockwise");
                telemetry.update();

            }

            while (error < -0.1) {
                error = zero + absTargetAngle - getCurrentZ();
                if(error > 180){
                    error -= 360;
                }
                if(error < -180){
                    error += 360;
                }
                double proportionalPower = error / dividend;
                proportionalPower = Math.abs(proportionalPower);

                if(proportionalPower < 0.1){
                    proportionalPower = 0.1;
                }

                //rotate right
                driveInDirection(proportionalPower, "ROTATE_RIGHT");

                //telemetry
                //            telemetry.addLine("currentZ" + getCurrentZ());
                telemetry.addLine("StartAngle: " + startAngle);
                telemetry.addLine("cumulativeZ" + getCurrentZ());
                telemetry.addLine("Error: " + error);
                telemetry.addLine("targetAngle: " + absTargetAngle);
                telemetry.addLine("rotation: clockwise");
                telemetry.update();
            }


            rightFrontDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftBackDrive.setPower(0);

        }
    }

    public void rotateToZLoc(double localTargetAngle){
        double dividend = 50;
        sleep(20);
        integratedHeading = 0;
        double startAngle = getCumulativeZ();
        double absTargetAngle = localTargetAngle + startAngle;
        double error = absTargetAngle - getCumulativeZ();
        while(Math.abs(error) > 1){
            while (error > 1) {


                error = absTargetAngle - getCumulativeZ();
                //rotate left
                driveInDirection(error / dividend, "ROTATE_LEFT");

                telemetry.addLine("TargetAngle: " + absTargetAngle);
                telemetry.addLine("cumulativeZ: " + getCumulativeZ());
                telemetry.addLine("Error: " + error);
                telemetry.addLine("rotation: counter clockwise");
                telemetry.update();
            }

            while (error < -1) {


                error = absTargetAngle - getCumulativeZ();
                //rotate right
                driveInDirection(error / dividend, "ROTATE_LEFT");

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

    public void alignToWallWithDistanceSensor(float threshold, float dividend){
        double dist1 = distance1.getDistance(DistanceUnit.CM);
        double dist2 = distance2.getDistance(DistanceUnit.CM);
        double error = dist1 - dist2;
        boolean close;
        boolean angleLeft;
        double i = 0;

        if(getCurrentZ() > -90){
            angleLeft = true;
        } else {
            angleLeft = false;
        }

        if(dist1 < 800 || dist2 < 800){
            close = true;
        } else {
            close = false;
        }


        while(i < 20){
            sleep(50);

            //TELEMETRY ERROR!!
            if((error > threshold || !close) && angleLeft == true) {
                while (error > threshold || !close) {
                    dist1 = distance1.getDistance(DistanceUnit.CM);
                    dist2 = distance2.getDistance(DistanceUnit.CM);
                    error = dist1 - dist2;
                    if(dist1 < 800 || dist2 < 800){
                        close = true;
                    } else {
                        close = false;
                    }
                    if (Math.abs(error / dividend) > 0.5 || !close) {
                        driveInDirection(0.5, "ROTATE_RIGHT");
                    } else {
                        if (error / dividend > 0.15) {
                            driveInDirection(error / dividend, "ROTATE_RIGHT");
                        } else {
                            driveInDirection(0.15, "ROTATE_RIGHT");
                        }
                    }
                    telemetry.addLine("Turning Right");
                    telemetry.addData("error:", error);
                    telemetry.update();
                }
            }else if((error < -threshold) && angleLeft == false) {
                while (error < -threshold) {
                    dist1 = distance1.getDistance(DistanceUnit.CM);
                    dist2 = distance2.getDistance(DistanceUnit.CM);
                    error = dist1 - dist2;
                    if(dist1 < 800 || dist2 < 800){
                        close = true;
                    } else {
                        close = false;
                    }
                    if (Math.abs(error / dividend) < -0.5 || !close) {
                        driveInDirection(-0.5, "ROTATE_RIGHT");
                    } else {
                        if (error / dividend < -0.15) {
                            driveInDirection(error / dividend, "ROTATE_RIGHT");
                        } else {
                            driveInDirection(-0.15, "ROTATE_RIGHT");
                        }
                    }
                    telemetry.addLine("Turning Left");
                    telemetry.addData("error:", error);
                    telemetry.update();
                }

            }

            i++;
        }
        driveInDirection(0, "STOP");
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

        integratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return integratedHeading;
    }

    //Test code for arm
    //Gets height in millimeters
    public void initArm() {
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(ARM_MIN_RANGE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("min pos",claw.MIN_POSITION);
        telemetry.addData("max pos", claw.MAX_POSITION);

        telemetry.addData("current height", getArmHeight());
        telemetry.update();
    }

    public boolean isClawOpen() {
        return !(claw.getPosition() > 0.1);

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
    public void armToHeight (double targetHeight){

        double currentHeight = getArmHeight();
        double error = targetHeight - currentHeight;

        if (targetHeight>currentHeight) {
            while (Math.abs(error) > 75) {
                telemetry.addData("error: ", error);
                telemetry.addData("Current Height: ", currentHeight);
                telemetry.update();
                armMotor.setPower(error / 125);
                currentHeight = getArmHeight();
                error = targetHeight - currentHeight;
            }
        } else {
            armMotor.setPower(0);
        }
    }

    public String senseColor(ColorSensor color){
        int red = color.red();
        int green = color.green();
        int blue = color.blue();

        if(red > green && red > blue){
            return "Red";
        } else if(green > blue){
            return "Green";
        } else {
            return "Blue";
        }

    }



}
