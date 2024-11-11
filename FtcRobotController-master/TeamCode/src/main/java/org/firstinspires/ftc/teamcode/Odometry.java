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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.util.Position;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name="Odometry", group="Autonomous")
public class Odometry extends LinearOpMode {

    // Declare OpMode objects
    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    //timer
    private final ElapsedTime timer = new ElapsedTime();

    @Override

    public void runOpMode() {



        // Find objects on Driver Controller
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right");

        int slow = 1;


        // Motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //TODO: Set the autonomous pose estimate to the matching trajectory
        drive.setPoseEstimate(new Pose2d(36,-60, Math.toRadians(90)));
        // pose estimate for blueright trajectory
        //drive.setPoseEstimate(new pose2d(-36,-60, Math.toRadians(90)));
        // pose estimate for blueleft trajectory
        //drive.setPoseEstimate(new pose2d(36,60, Math.toRadians(180)));
        // pose estimate for redleft trajectory
        //drive.setPoseEstimate(new pose2d(-36,60, Math.toRadians(180)));
        // pose estimate for redright trajectory


        /*

        ╔══/═══════════════════════════════\═╗
        ╠═/                 ↑  +x           \║
        ║                       +y           ║
        ║ █ █ █        /╔══╗\   →      █ █ █ ║
        ║             / ║██║ \               ║
        ║             \ ║██║ /               ║
        ║ █ █ █        \╚══╝/          █ █ █ ║
        ║                                    ║
        ║\                                 /═╣
        ╚═\═══════════════════════════════/══╝
        Go to https://docs.google.com/document/d/1GJCBK_APcPKTCdh_Iw3J0L3xCFaDmAAdjnVdM-0Za-Y/edit?pli=1&tab=t.1vz37yah5nt#heading=h.dlpsbpb5k77b
        to find out how the trajectories work.
        */

        Trajectory blueright = drive.trajectoryBuilder(new Pose2d(36, -60, Math.toRadians(90)))
                .lineTo(new Vector2d(0,-21))
                .lineTo(new Vector2d(0,-25))
                .lineTo(new Vector2d(48,-25))
                .lineTo(new Vector2d(-54,-54))
                .lineTo(new Vector2d(-36,60))
                .lineTo(new Vector2d(-54,-54))
                .lineTo(new Vector2d(-36,72))
                .lineTo(new Vector2d(-54,-54))
                .build();

        Trajectory redleft = drive.trajectoryBuilder(new Pose2d(36,60, Math.toRadians(180)))
                .lineTo(new Vector2d(0,21))
                .lineTo(new Vector2d(0,25))
                .lineTo(new Vector2d(48,25))
                .lineTo(new Vector2d(54,54))
                .lineTo(new Vector2d(-36,-60))
                .lineTo(new Vector2d(54,54))
                .lineTo(new Vector2d(-36,-72))
                .lineTo(new Vector2d(54,54))
                .build();

        waitForStart();
        if (isStopRequested()) return;
        //TODO: Change the followed trajectory to match its position on the field
        drive.followTrajectory(blueright);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }

}