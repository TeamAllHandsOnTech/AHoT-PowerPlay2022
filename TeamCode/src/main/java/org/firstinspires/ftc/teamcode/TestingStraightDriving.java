package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.DriveDirections;


@Autonomous(name="TestingStraightDriving", group="Hazard")
public class TestingStraightDriving extends DriveDirections
{


    @Override
    public void runOpMode(){

        isHazard = true;

        super.runOpMode();

        initArm();

        waitForStart();


        StraightDrive(0.5, 0.65, "FORWARD");
        StraightDrive(0.5, 0.65, "RIGHT");
        StraightDrive(0.5, 0.65, "BACKWARD");
        StraightDrive(0.5, 0.65, "LEFT");


        while (opModeIsActive()){

        }
    }


   }