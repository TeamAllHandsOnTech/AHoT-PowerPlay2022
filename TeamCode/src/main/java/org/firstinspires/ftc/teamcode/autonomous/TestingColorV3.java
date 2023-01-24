package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DriveDirections;

@TeleOp
public class TestingColorV3 extends DriveDirections {
    // Define a variable for our color sensor
    ColorSensor frontColor;
    ColorSensor backColor;

    @Override
    public void runOpMode() {

        super.runOpMode();
        // Get the color sensor from hardwareMap
        frontColor = hardwareMap.get(ColorSensor.class, "frontColor");
        backColor = hardwareMap.get(ColorSensor.class, "backColor");

        // Wait for the Play button to be pressed
        waitForStart();



        while(frontColor.blue() <= 300 || backColor.blue() <= 300){
            //Drive Right
            driveInDirection(0.1, "RIGHT");
            if(frontColor.blue() > 300 && backColor.blue() <= 300){
                //Rotate Left
                driveInDirection(0.1, "ROTATE_LEFT");
            } else if(frontColor.blue() > 300 && backColor.blue() <= 300){
                // Rotate Right
                driveInDirection(0.1, "ROTATE_RIGHT");
            }
        }

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {
            telemetry.addData("red", frontColor.red());
            telemetry.addData("green", frontColor.green());
            telemetry.addData("blue", frontColor.blue());
            telemetry.addData("Color", senseColor(frontColor));
            telemetry.update();
        }
    }
}