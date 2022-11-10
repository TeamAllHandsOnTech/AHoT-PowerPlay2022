package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DriveDirections;


@Autonomous(name="VisionTest", group="Trauma")

public class VisionMk1 extends DriveDirections {

    private ElapsedTime runtime = new ElapsedTime();
    private double moveSpeed = 0.3;


    @Override
    public void runOpMode() {
        super.runOpMode();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //Pushes cone onto junction
        switch (fakeVision()){
            case 1:
                DriveForDistance(0.5, 0.4, "LEFT");
                break;
            case 2:
                break;
            case 3:
                DriveForDistance(0.5, 1, "RIGHT");
                break;
            default:
                break;
        }
        DriveForDistance(0.5, 0.9, "FORWARD");

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {




            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

