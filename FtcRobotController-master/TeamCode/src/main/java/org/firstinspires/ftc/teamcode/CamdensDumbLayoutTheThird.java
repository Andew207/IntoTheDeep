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


// Importing things
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Camden's Dumb Layout The Third", group="Linear Opmode")
public class CamdensDumbLayoutTheThird extends LinearOpMode {

    // Declare OpMode objects
    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor motorForSucking = null;
    private DcMotor liftMotor = null;
    private DcMotor fake = null;
    private CRServo swing = null;
    private DcMotor fly = null;
    private Servo airplane = null;
    private TouchSensor limit = null;
    private Servo score = null;

    private CRServo cr1 = null;
    private CRServo cr2 = null;
    private CRServo cr3 = null;

    //timer
    private final ElapsedTime timer = new ElapsedTime();

    @Override

    public void runOpMode() {



        // Find objects on Driver Controller
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right");
        airplane = hardwareMap.get(Servo.class, "airplane");
        liftMotor = hardwareMap.get(DcMotor.class, "lift");
        limit = hardwareMap.get(TouchSensor.class, "limit");
        fly = hardwareMap.get(DcMotor.class, "fly");
        motorForSucking = hardwareMap.get(DcMotor.class, "suck");
        swing = hardwareMap.get(CRServo.class, "swing");
        fake = hardwareMap.get(DcMotor.class, "this doesnt exist");

        cr1 = hardwareMap.get(CRServo.class, "cr1");
        cr2 = hardwareMap.get(CRServo.class, "cr2");
        cr3 = hardwareMap.get(CRServo.class, "cr3");

        score = hardwareMap.get(Servo.class, "score");
        int slow = 1;

        // initial servo positions
        score.setPosition(1);
        airplane.setPosition(0);

        // Motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        // Reset encoders
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Run with encoder
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Declare random variables
        boolean changed = false;
        boolean changed1 = false;
        boolean changed2 = true;
        boolean changed3 = true;
        boolean changed4 = true;
        double suck = 0;
        double drive;
        double strafe;
        double turn;
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        double liftPos = 0;
        int liftTargPos = 0;



        while (opModeIsActive()) {

            liftPos = liftMotor.getCurrentPosition();

            // Drive variables
            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn  =  gamepad1.right_stick_x/2;

            // Setting the 3 intake servos
            if (gamepad2.left_trigger != 0){
                cr1.setPower(-1);
                cr2.setPower(1);
                cr3.setPower(1);
            }
            else if (gamepad2.right_trigger !=0){
                cr1.setPower(1);
                cr2.setPower(-1);
                cr3.setPower(-1);
            }
            else {
                    cr1.setPower(0);
                    cr2.setPower(0);
                    cr3.setPower(0);
            }

            // Lift moter
            if (limit.isPressed() && changed1 == false){
                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftMotor.setPower(0);
                changed1 = true;
                liftTargPos = 0;
            }
            else if ((gamepad2.b && changed3) && !limit.isPressed() && liftTargPos < 400){
                liftTargPos += 220;
            }

            else if((gamepad2.y && changed2) && liftTargPos > -2500 && !limit.isPressed()){
                liftTargPos -= 220;
            }
            else if((gamepad2.y && changed2) && limit.isPressed()){
                liftTargPos -= 750;
            }
            if (gamepad2.y)
                changed2 = false;
            if (gamepad2.b)
                changed3 = false;
            if (gamepad2.a && changed4){
                liftTargPos += 1000;
                changed4 = false;
            }

            if (liftMotor.getCurrentPosition() < -50){
                changed1 = false;
            }
            if (!gamepad2.y)
                changed2 = true;
            if (!gamepad2.b)
                changed3 = true;
            if (!gamepad2.a)
                changed4 = true;
            // Scoring servo
            if (gamepad2.left_bumper){
                score.setPosition(0);
            }
            else if (gamepad2.right_bumper) {
                score.setPosition(0.25);
            }
            else score.setPosition(1);

            // slow mode! //
            if(gamepad1.a && !changed) {
                if(slow == 1) slow = 2;
                else slow = 1;
                changed = true;
            } else if(!gamepad1.a) changed = false;

            // Intake
            suck = gamepad2.left_stick_x;

            // Airplane launching
            if(gamepad2.x && gamepad1.x) {
                airplane.setPosition(1);
            }
            if(gamepad1.y) airplane.setPosition(0);


            // Drive equations
            frontLeftPower    = Range.clip((drive + strafe + turn)/slow, -0.75, 0.75);
            frontRightPower   = Range.clip((drive - strafe - turn)/slow, -0.75, 0.75);
            backLeftPower    = Range.clip((drive - strafe + turn)/slow, -0.75, 0.75);
            backRightPower   = Range.clip((drive + strafe - turn)/slow, -0.75, 0.75);

            // Tape measure
            fly.setPower(gamepad1.left_trigger-gamepad1.right_trigger);

            // Swinging the scoring mechanism
            int swing1 = 0;
            if (gamepad2.dpad_up && !gamepad2.dpad_down){
                swing1 = 1;
            }
            else if (gamepad2.dpad_down && !gamepad2.dpad_up){
                swing1 = -1;
            }
            else swing1 = 0;
            swing.setPower(swing1);

            // Set powers
            if (!gamepad1.b) {
                frontLeftDrive.setPower(frontLeftPower);
                backLeftDrive.setPower(backLeftPower);
                frontRightDrive.setPower(frontRightPower);
                backRightDrive.setPower(backRightPower);
            }
            else{
                frontLeftDrive.setPower(-0.15);
                frontRightDrive.setPower(-0.15);
                backRightDrive.setPower(-0.15);
                backLeftDrive.setPower(-0.15);
            }
            motorForSucking.setPower(suck);


            liftMotor.setTargetPosition(liftTargPos);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (limit.isPressed() && !gamepad2.y)
                liftMotor.setPower(0);
            else liftMotor.setPower(-1);

            // TELEMETRY
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("FL Encoder", frontLeftDrive.getCurrentPosition());
            telemetry.addData("FR Encoder", frontRightDrive.getCurrentPosition());
            telemetry.addData("BL Encoder", backLeftDrive.getCurrentPosition());
            telemetry.addData("BR Encoder", backRightDrive.getCurrentPosition());
            telemetry.addData("Changed", changed1);
            telemetry.addData("limit", limit.isPressed());
            telemetry.addData("Servo Position", airplane.getPosition());
            telemetry.addData("lift pos", liftMotor.getCurrentPosition());
            telemetry.addData("Lift", liftTargPos);
            telemetry.addData("lift power", liftMotor.getPower());
            telemetry.addData("lift mode", liftMotor.getMode());
            telemetry.addData("airplane", airplane.getPosition());
            telemetry.addData("Lift encoder", liftPos);
            telemetry.addData("X", gamepad2.x && gamepad1.x);
            telemetry.addData("FL Power", frontLeftPower);
            telemetry.addData("FR Power", frontRightPower);
            telemetry.addData("BL Power", backLeftPower);
            telemetry.addData("BR Power", backRightPower);

            telemetry.addData("Bumper Pressed?", gamepad2.left_bumper);
            telemetry.addData("servo direction", score.getDirection());

            telemetry.update();
        }
    }

}