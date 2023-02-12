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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.DriveDirections;


@TeleOp(name="TraumaTeleOp", group="Hazard")


public class TestingInputs extends DriveDirections {
    double zero = 0;
    double powerLevel = 0.8;
    double deadZone = 0.5;

    ColorSensor frontColor;
    ColorSensor backColor;

    public DistanceSensor distance1 = null;
    public DistanceSensor distance2 = null;

    public String pickColor(ColorSensor sensor, double sensitivity) {
        if (sensor.red()>sensitivity*(sensor.green()+sensor.blue())) {return "Red";}
        else if (sensor.green()>sensitivity*(sensor.blue()+sensor.red())) {return "Green";}
        else if (sensor.blue()>sensitivity*(sensor.red()+sensor.green())) {return "Blue";}
        else {return "Grey";}
    }

    @Override
    public void runOpMode() {

        frontColor = hardwareMap.get(ColorSensor .class, "frontColor");
        backColor = hardwareMap.get(ColorSensor.class, "backColor");

        distance1 = hardwareMap.get(DistanceSensor .class, "Distance1");
        distance2 = hardwareMap.get(DistanceSensor.class, "Distance2");

        isHazard = false;
        super.runOpMode();

        initArm();
        waitForStart();

        while (opModeIsActive()) {
            /**GAMEPAD 1**/
            //slow down power if bumper is pressed
            telemetry.addLine("Front color: "+pickColor(frontColor,1.1));
            telemetry.addLine("Front color: "+pickColor(frontColor,1.1));
            telemetry.addData("distance 1", distance1.getDistance(DistanceUnit.MM));
            telemetry.addData("distance 2", distance2.getDistance(DistanceUnit.MM));
            telemetry.addData("angle", (90 / Math.PI) * (Math.atan((distance1.getDistance(DistanceUnit.MM) - distance2.getDistance(DistanceUnit.MM)) / 85)));
            telemetry.update();
        }
    }
}