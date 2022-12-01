package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DriveDirections;


@Autonomous(name="Straight Drive", group="Hazard")

public class TestingStraightDrive extends DriveDirections {

    private ElapsedTime runtime = new ElapsedTime();
    private double moveSpeed = 0.3;


    @Override
    public void runOpMode() {

        isHazard = true;

        super.runOpMode();


//
        waitForStart();
        runtime.reset();

        rotateToZLoc(90, 50);
//        StraightDrive(0.2, 2, "LEFT", 0.5);
//        DriveInDirection(0, "FORWARD");
        rotateToZLoc(270, 50);
        rotateToZLoc(360, 50);
        rotateToZLoc(-360, 50);
        StraightDrive(0.5, 1.5, "RIGHT");


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {




            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

