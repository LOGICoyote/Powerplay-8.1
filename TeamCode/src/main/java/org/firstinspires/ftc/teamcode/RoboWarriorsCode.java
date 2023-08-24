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


package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp

public class RoboWarriorsCode extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left = null;
    private DcMotor right = null;

    private Servo claw;
    private Servo arm;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        left = hardwareMap.get(DcMotor.class, "motor00");
        right = hardwareMap.get(DcMotor.class, "motor01");

        claw=hardwareMap.get(Servo.class,"claw");
        arm =hardwareMap.get(Servo.class,"arm");
        left.setDirection(DcMotorSimple.Direction.REVERSE);



        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void start() {

        runtime.reset();
    }

    @Override
    public void loop() {
        double left_YAxis;
        double left_XAxis;
        double right_YAxis;
        double right_XAxis;
        double left_trigger;
        double right_trigger;

        left_YAxis = -gamepad1.left_stick_y;
        left_XAxis = gamepad1.left_stick_x;
        right_YAxis = -gamepad1.right_stick_y;
        right_XAxis = gamepad1.right_stick_x;
        left_trigger = gamepad1.left_trigger;
        right_trigger = gamepad1.right_trigger;

        if (left_trigger > 0) {
            turnleft(left_trigger);

        } else if (right_trigger > 0) {
            turnright(right_trigger);
        }
        else if (left_YAxis > 0.2) {
            goforward(left_YAxis);

        } else if (left_YAxis < -0.2) {
            gobackward(left_YAxis);

        } else
            gostop(0);

        if (gamepad1.y){
            arm.setPosition(.4);
        }
        if (gamepad1.a){
            arm.setPosition(.5);
        }
        if (gamepad1.b){
            claw.setPosition(.5);
        }
        if (gamepad1.x){
            claw.setPosition(0);
        }



    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private void gostop(double speed) {
        left.setPower(0);
        right.setPower(0);

    }
    private void goforward(double speed) {
        left.setPower(speed);
        right.setPower(speed);
    }
    private void gobackward(double speed) {
        left.setPower(speed);
        right.setPower(speed);
    }

    private void turnleft(double speed) {
        left.setPower(-speed );
        right.setPower(speed );
    }
    private void turnright(double speed) {
        left.setPower(speed );
        right.setPower(-speed );
    }

}