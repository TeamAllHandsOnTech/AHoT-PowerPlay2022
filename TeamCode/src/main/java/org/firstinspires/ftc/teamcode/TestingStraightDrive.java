package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Straight Drive", group="Hazard")

public class TestingStraightDrive extends DriveDirections {

    private ElapsedTime runtime = new ElapsedTime();
    private double moveSpeed = 0.3;


    @Override
    public void runOpMode() {

        super.runOpMode();


//
        waitForStart();
        runtime.reset();

        rotateToZLoc(90);
//        StraightDrive(0.2, 2, "LEFT", 0.5);
//        DriveInDirection(0, "FORWARD");
        rotateToZLoc(270);
        rotateToZLoc(360);
        rotateToZLoc(-360);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {




            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

