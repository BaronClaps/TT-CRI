package org.firstinspires.ftc.teamcode;
/* Copyright (c) 2023 FIRST. All rights reserved.
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


import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Subsystems.ArmRotationSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.PresetSubsystem;

import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@Autonomous(name = "CloseRed", group = "Autonomous")

public class CloseRed extends LinearOpMode {

    private HuskyLens huskyLens;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(60, 12, Math.toRadians(90)));
        ClawSubsystem.claw claw = new ClawSubsystem.claw(hardwareMap);
        ArmSubsystem.arm arm = new ArmSubsystem.arm(hardwareMap);
        ArmRotationSubsystem.armRotation armROT = new ArmRotationSubsystem.armRotation(hardwareMap);
        PresetSubsystem.presets presets = new PresetSubsystem.presets(hardwareMap);

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        Action purpleTrajectoryAction1;
        Action purpleTrajectoryAction2;
        Action purpleTrajectoryAction3;
        Action yellowTrajectoryAction1;
        Action yellowTrajectoryAction2;
        Action yellowTrajectoryAction3;
        Action parkingTrajectoryAction1;
        Action parkingTrajectoryAction2;
        Action parkingTrajectoryAction3;

        //This action drives to the first tape line
        purpleTrajectoryAction1 = drive.actionBuilder(drive.pose)
                .waitSeconds(.5)
                .strafeTo(new Vector2d(26, 38.5))
                .waitSeconds(.1)
                .turn(-1 * PI / 2)
                .waitSeconds(.1)
                .build();

        //This action drives to the second tape line
        purpleTrajectoryAction2 = drive.actionBuilder(drive.pose)
                .waitSeconds(.5)
                .strafeTo(new Vector2d(56.5, 16))
                .waitSeconds(.1)
                .strafeTo(new Vector2d(37.5, 16))
                .waitSeconds(.1)
                .build();

        //This action drives to the third tape line
        purpleTrajectoryAction3 = drive.actionBuilder(drive.pose)
                .waitSeconds(.5)
                .strafeTo(new Vector2d(29.5, 40))
                .waitSeconds(.1)
                .turn(PI / 2)
                .waitSeconds(.1)
                .build();

        //This action drives to the first backdrop section
        yellowTrajectoryAction1 = drive.actionBuilder(drive.pose)
                .waitSeconds(.1)
                .strafeTo(new Vector2d(22.5, 48))
                .waitSeconds(.1)
                .build();

        //This action drives to the second backdrop section
        yellowTrajectoryAction2 = drive.actionBuilder(drive.pose)
                .waitSeconds(.1)
                .turn(PI / 2)
                .waitSeconds(.1)
                .strafeTo(new Vector2d(27, 48))
                .waitSeconds(.1)
                .build();

        //This action drives to the third backdrop section
        yellowTrajectoryAction3 = drive.actionBuilder(drive.pose)
                .waitSeconds(.1)
                .strafeTo(new Vector2d(43.5, 48))
                .waitSeconds(.1)
                .strafeTo(new Vector2d(43.5,55))
                .waitSeconds(.1)
                .build();

        //This action drives to robot to the first parking zone
        parkingTrajectoryAction1 = drive.actionBuilder(drive.pose)
                .waitSeconds(.1)
                .strafeTo(new Vector2d(36,40))
                .waitSeconds(.1)
                .strafeTo((new Vector2d(66, 54)))
                .waitSeconds(.1)
                .build();

        //This action drives to robot to the second parking zone
        parkingTrajectoryAction2 = drive.actionBuilder(drive.pose)
                .waitSeconds(.1)
                .strafeTo(new Vector2d(36,40))
                .waitSeconds(.1)
                .strafeTo((new Vector2d(60.5, 50)))
                .waitSeconds(.1)
                .build();

        //This action drives to robot to the third parking zone
        parkingTrajectoryAction3 = drive.actionBuilder(drive.pose)
                .waitSeconds(.1)
                .strafeTo(new Vector2d(36,40))
                .waitSeconds(.1)
                .strafeTo((new Vector2d(67.5, 50)))
                .waitSeconds(.1)
                .build();


        Actions.runBlocking(claw.closeClaws());
        Actions.runBlocking(presets.resetEncoders());

        Deadline rateLimit = new Deadline(1, TimeUnit.SECONDS); //from huskylens example
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }//makes sure the huskylens is talking to the control hub
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);// can change to other algorithms


        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {

            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();// from huskylens
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());// this gives you the data
                telemetry.addData("location?", blocks[i].x);// this gives you just x
                //TODO ensure your x values of the husky lens are appropriate to the desired areas
                //----------------------------1----------------------------\\
                if (blocks[i].x < 90 && blocks[i].id == 1) {
                    Actions.runBlocking(
                            new SequentialAction(
                                    presets.startPos(),
                                    purpleTrajectoryAction1,
                                    claw.openRClaw(),
                                    presets.scoringPos(),
                                    yellowTrajectoryAction1,
                                    claw.openLClaw(),
                                    parkingTrajectoryAction1,
                                    presets.groundPos()
                            )
                    );
                    sleep(400000);
                }


                //----------------------------2----------------------------\\
                if (blocks[i].x > 90 && blocks[i].x < 180 && blocks[i].id == 1) {
                    Actions.runBlocking(
                            new SequentialAction(
                                    presets.startPos(),
                                    purpleTrajectoryAction2,
                                    claw.openRClaw(),
                                    presets.scoringPos(),
                                    yellowTrajectoryAction2,
                                    claw.openLClaw(),
                                    parkingTrajectoryAction2,
                                    presets.groundPos()
                            )
                    );
                    sleep(400000);
                }


                //----------------------------3---------------------------\\
                if (blocks[i].x > 180 && blocks[i].id == 1) {
                    Actions.runBlocking(
                            new SequentialAction(
                                    presets.startPos(),
                                    purpleTrajectoryAction3,
                                    claw.openRClaw(),
                                    presets.scoringPos(),
                                    yellowTrajectoryAction3,
                                    claw.openLClaw(),
                                    parkingTrajectoryAction3,
                                    presets.groundPos()
                            )
                    );
                    sleep(400000);
                }


            }
        }
    }
}

