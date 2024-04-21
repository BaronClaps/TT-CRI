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
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClawSubsystem;

import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name = "CB_Blue", group = "Autonomous")
public class CB_Blue extends LinearOpMode {
    private HuskyLens huskyLens;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-60,12, 0));
        ClawSubsystem.claw claw = new ClawSubsystem.claw(hardwareMap);
        ArmSubsystem.arm arm = new ArmSubsystem.arm(hardwareMap);



        Action purpletrajectoryAction1;
        Action purpletrajectoryAction2;
        Action purpletrajectoryAction3;
        Action yellowtrajectoryAction1;
        Action yellowtrajectoryAction2;
        Action yellowtrajectoryAction3;


        purpletrajectoryAction1 = drive.actionBuilder(drive.pose)

                .build();
        purpletrajectoryAction2 = drive.actionBuilder(drive.pose)

                .build();
        purpletrajectoryAction3 = drive.actionBuilder(drive.pose)

                .build();
        yellowtrajectoryAction1 = drive.actionBuilder(drive.pose)
                
                .build();
        yellowtrajectoryAction2 = drive.actionBuilder(drive.pose)

                .build();
        yellowtrajectoryAction3 = drive.actionBuilder(drive.pose)

                .build();


        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.closeClaws(),
                            claw.closeClaws()
        );

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

        if (isStopRequested()) return;

        Action purpleTrajectoryActionChosen = null;
        Action yellowTrajectoryActionChosen = null;

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
                if (blocks[i].x < 100 && blocks[i].id == 2) {
                    purpleTrajectoryActionChosen = purpletrajectoryAction1;
                    yellowTrajectoryActionChosen = yellowtrajectoryAction1;
                }


                //----------------------------2----------------------------\\
                if (blocks[i].x > 100 && blocks[i].x < 180 && blocks[i].id == 2) {
                    purpleTrajectoryActionChosen = purpletrajectoryAction2;
                    yellowTrajectoryActionChosen = yellowtrajectoryAction2;
                }


                //----------------------------3---------------------------\\
                if (blocks[i].x > 180 && blocks[i].id == 2) {
                    purpleTrajectoryActionChosen = purpletrajectoryAction3;
                    yellowTrajectoryActionChosen = yellowtrajectoryAction3;
                }
            }
        }

        Actions.runBlocking(
                new SequentialAction(
                        purpleTrajectoryActionChosen,
                        yellowTrajectoryActionChosen
                )
        );
    }
}