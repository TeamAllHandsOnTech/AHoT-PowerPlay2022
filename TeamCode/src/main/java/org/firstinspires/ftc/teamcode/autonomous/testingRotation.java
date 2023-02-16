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

package org.firstinspires.ftc.teamcode.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.DriveDirections;


@TeleOp(name="TestingRotation", group="a")
public class testingRotation extends DriveDirections {


    @Override
    public void runOpMode() {
        super.runOpMode();
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();

        distance1 = hardwareMap.get(DistanceSensor.class, "Distance1");
        distance2 = hardwareMap.get(DistanceSensor.class, "Distance2");


        waitForStart();

        rotateToZAbs(-80, 0);

        sleep(1000);

        double dist1 = distance1.getDistance(DistanceUnit.MM);
        double dist2 = distance2.getDistance(DistanceUnit.MM);
        double thresA = 3;
        double errorA = (90/Math.PI)*Math.atan((dist1-dist2)/85);

        while(Math.abs(errorA) > thresA){
            dist1 = distance1.getDistance(DistanceUnit.MM);
            dist2 = distance2.getDistance(DistanceUnit.MM);
            errorA = (90/Math.PI)*Math.atan((dist1-dist2)/85);

            driveInDirection(Math.signum(errorA)*0.2, "ROTATE_RIGHT");
        }

        driveInDirection(0,"STOP");


        sleep(5000);

        while (opModeIsActive()) {
            telemetry.addData("Left Dist", distance1.getDistance(DistanceUnit.CM));
            telemetry.addData("Right Dist", distance2.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
