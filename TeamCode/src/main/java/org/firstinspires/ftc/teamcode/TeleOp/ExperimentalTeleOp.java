/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DriveDirections;


@TeleOp(name="ExperimentalTeleOp", group="Z")


public class ExperimentalTeleOp extends DriveDirections {
    double zero = 0;
    double powerLevel = 0.8;
    double deadZone = 0.5;

    @Override
    public void runOpMode() {
        isHazard = true;
        super.runOpMode();



        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        double lfPower = 0;
        double lbPower = 0;
        double rfPower = 0;
        double rbPower = 0;

        double lfPowerStrafe = 0;
        double lbPowerStrafe = 0;
        double rfPowerStrafe = 0;
        double rbPowerStrafe = 0;

        double lfPowerForwards = 0;
        double lbPowerForwards = 0;
        double rfPowerForwards = 0;
        double rbPowerForwards = 0;

        double lfPowerRotate = 0;
        double lbPowerRotate = 0;
        double rfPowerRotate = 0;
        double rbPowerRotate = 0;

        initArm();
        waitForStart();

        while (opModeIsActive()) {
            /**GAMEPAD 1**/
            //slow down power if bumper is pressed
            if (gamepad1.left_bumper) {
                powerLevel = 0.3;
            } else if (gamepad1.right_bumper) {
                powerLevel = 0.6;
            }else if (gamepad1.left_stick_button) {
                powerLevel = 1;
            }else {
                powerLevel = 0.8;
            }

            if(Math.abs(gamepad1.left_stick_x) > deadZone || Math.abs(gamepad1.left_stick_y) > deadZone){
                lfPowerStrafe = gamepad1.left_stick_x * powerLevel;
                lbPowerStrafe = -gamepad1.left_stick_x * powerLevel;
                rfPowerStrafe = -gamepad1.left_stick_x * powerLevel;
                rbPowerStrafe = gamepad1.left_stick_x * powerLevel;

                lfPowerForwards = -gamepad1.left_stick_y * powerLevel;
                lbPowerForwards = -gamepad1.left_stick_y * powerLevel;
                rfPowerForwards = -gamepad1.left_stick_y * powerLevel;
                rbPowerForwards = -gamepad1.left_stick_y * powerLevel;
            }

            if(Math.abs(gamepad1.right_stick_x) > deadZone){
                lfPowerRotate = gamepad1.right_stick_x * powerLevel;
                lbPowerRotate = gamepad1.right_stick_x * powerLevel;
                rfPowerRotate = -gamepad1.right_stick_x * powerLevel;
                rbPowerRotate = -gamepad1.right_stick_x * powerLevel;
            }

            lfPower = (lfPowerStrafe + lfPowerForwards + lfPowerRotate) / 3;
            lbPower = (lbPowerStrafe + lbPowerForwards + lbPowerRotate) / 3;
            rfPower = (rfPowerStrafe + rfPowerForwards + rfPowerRotate) / 3;
            rbPower = (rbPowerStrafe + rbPowerForwards + rbPowerRotate) / 3;

            leftFrontDrive.setPower(lfPower);
            leftBackDrive.setPower(lbPower);
            rightFrontDrive.setPower(rfPower);
            rightBackDrive.setPower(rbPower);

            if(gamepad1.y){
                resetZero(0);
            }
            if(gamepad1.a){
                resetZero(180);
            }
            if(gamepad1.x){
                resetZero(90);
            }
            if(gamepad1.b){
                resetZero(-90);
            }

            if(gamepad1.dpad_up){
                rotateToZAbs(0, zero);
            } else if(gamepad1.dpad_left){
                rotateToZAbs(90, zero);
            } else if(gamepad1.dpad_right){
                rotateToZAbs(-90, zero);
            } else if(gamepad1.dpad_down){
                rotateToZAbs(180, zero);
            }

            /**GAMEPAD 2**/


            //Distances have not been learned yet

//            double targetHeight = 0;
//            boolean setHeight = false;
//            if (gamepad2.dpad_down) { //Ground Junction
//                targetHeight=1;
//                setHeight = true;
//                //1
//            } else if (gamepad2.dpad_left) { //Low Junction
//                targetHeight=346;
//                setHeight = true;
//                //346
//            } else if (gamepad2.dpad_right) { //Medium  Junction
//                targetHeight=600;
//                setHeight = true;
//                //600
//            } else if (gamepad2.dpad_up) { //High  Junction
//                targetHeight=854;
//                setHeight = true;
//                //854
//            }
//            //Arm to height code doesn't work currently

//            double currentHeight = getArmHeight();
//            double error = targetHeight + currentHeight;
//
//            if (setHeight==true && Math.abs(error)>10) {
//                telemetry.addData("error: ", error);
//                telemetry.addData("Current Height: ", currentHeight);
//                telemetry.update();
//                armMotor.setPower(error / 100);
//            } else {
//                setHeight = false;
//              }

                double armPower = 0;
                if (Math.abs(gamepad2.left_stick_y) > 0.1){
                    armPower = gamepad2.left_stick_y*0.75;
                } else {
                    armPower = 0;
                }
                if (Math.abs(gamepad2.right_stick_y) > 0.1){
                    armPower = gamepad2.right_stick_y*0.9;
                }
                armMotor.setPower(-armPower);

            if(gamepad2.right_bumper){
                closeClaw();

            }else if (!isClawOpen()){
                openClaw();
            }
            if(gamepad2.left_bumper){
                armStop();
            }
        }
        telemetry.addLine("heading: " + getCurrentZ());
        telemetry.update();
    }
    void resetZero(double degreeOffZero){
        zero = getCurrentZ() - degreeOffZero;
    }
}