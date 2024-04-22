package org.firstinspires.ftc.teamcode;
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

@Config
@Autonomous(name = "Close Red", group = "Autonomous")
public class CloseRed extends LinearOpMode {
    private HuskyLens huskyLens;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(60,12, Math.toRadians(180)));
        ClawSubsystem.claw claw = new ClawSubsystem.claw(hardwareMap);
        ArmSubsystem.arm arm = new ArmSubsystem.arm(hardwareMap);
        ArmRotationSubsystem.armRotation armROT = new ArmRotationSubsystem.armRotation(hardwareMap);
        PresetSubsystem.presets presets = new PresetSubsystem.presets(hardwareMap);



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
                .strafeTo(new Vector2d(-26, 38.5))
                .waitSeconds(.1)
                .turn(-1*PI/2)
                .waitSeconds(.1)
                .build();

        //This action drives to the second tape line
        purpleTrajectoryAction2 = drive.actionBuilder(drive.pose)
                .waitSeconds(.5)
                .lineToX(-56.5)
                .waitSeconds(.1)
                .strafeTo(new Vector2d(-37.5, 16))
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
                .lineToY(55)
                .waitSeconds(.1)
                .build();

        //This action drives to robot to the first parking zone
        parkingTrajectoryAction1 = drive.actionBuilder(drive.pose)
                .waitSeconds(.1)
                .lineToY(40)
                .waitSeconds(.1)
                .strafeTo((new Vector2d(66, 54)))
                .waitSeconds(.1)
                .build();

        //This action drives to robot to the second parking zone
        parkingTrajectoryAction2 = drive.actionBuilder(drive.pose)
                .waitSeconds(.1)
                .lineToY(40)
                .waitSeconds(.1)
                .strafeTo((new Vector2d(60.5, 50)))
                .waitSeconds(.1)
                .build();

        //This action drives to robot to the third parking zone
        parkingTrajectoryAction3 = drive.actionBuilder(drive.pose)
                .waitSeconds(.1)
                .lineToY(40)
                .waitSeconds(.1)
                .strafeTo((new Vector2d(67.5, 50)))
                .waitSeconds(.1)
                .build();


        //Actions that need to happen on initialization; for instance, a claw tightening.
        Actions.runBlocking(claw.closeClaws());

        //From HuskyLens example
        Deadline rateLimit = new Deadline(1, TimeUnit.SECONDS);
        rateLimit.expire();

        //Checks if HuskyLens is talking to the Control Hub
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        //Can change to other algorithms
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);


        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action purpleTrajectoryActionChosen = null;
        Action yellowTrajectoryActionChosen = null;
        Action parkingTrajectoryActionChosen = null;

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
                    purpleTrajectoryActionChosen = purpleTrajectoryAction1;
                    yellowTrajectoryActionChosen = yellowTrajectoryAction1;
                    parkingTrajectoryActionChosen = parkingTrajectoryAction1;
                }


                //----------------------------2----------------------------\\
                if (blocks[i].x > 90 && blocks[i].x < 180 && blocks[i].id == 1) {
                    purpleTrajectoryActionChosen = purpleTrajectoryAction2;
                    yellowTrajectoryActionChosen = yellowTrajectoryAction2;
                    parkingTrajectoryActionChosen = parkingTrajectoryAction2;
                }


                //----------------------------3---------------------------\\
                if (blocks[i].x > 180 && blocks[i].id == 1) {
                    purpleTrajectoryActionChosen = purpleTrajectoryAction3;
                    yellowTrajectoryActionChosen = yellowTrajectoryAction3;
                    parkingTrajectoryActionChosen = parkingTrajectoryAction3;
                }
            }
        }

        Actions.runBlocking(
                new SequentialAction(
                        presets.startPos(),
                        purpleTrajectoryActionChosen,
                        claw.openRClaw(),
                        presets.scoringPos(),
                        yellowTrajectoryActionChosen,
                        claw.openLClaw(),
                        parkingTrajectoryActionChosen,
                        presets.groundPos()
                )
        );
    }
}