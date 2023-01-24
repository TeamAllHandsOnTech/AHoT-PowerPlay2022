package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="TestingStraightDriving", group="Hazard")
public class TestingStraightDriving extends DriveDirections
{


    @Override
    public void runOpMode(){

        super.runOpMode();

        initArm();

        waitForStart();


        straightDrive(0.5, 0.65, "FORWARD");
        straightDrive(0.5, 0.65, "RIGHT");
        straightDrive(0.5, 0.65, "BACKWARD");
        straightDrive(0.5, 0.65, "LEFT");


        while (opModeIsActive()){

        }
    }


   }