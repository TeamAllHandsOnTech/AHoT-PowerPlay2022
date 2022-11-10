package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DriveDirections;


@Autonomous(name="VisionFakeTest", group="Trauma")

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


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {




            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

