package org.firstinspires.ftc.teamcode;
/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TTDriveCode", group="Linear Opmode")
public class TTDriveCode extends LinearOpMode {

    //---------------Declare Hardware Variables-----------------------//

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor arm = null;
    private Servo airplane = null;
    private DcMotor armROT = null;
    private Servo clawLeft = null;
    private Servo clawRight = null;
    private Servo clawRotate = null;

    //---------------Declare Variables-----------------------//

    private int bspeed;
    private double rfSpeed;
    private double lfSpeed;
    private double rbSpeed;
    private double lbSpeed;
    //---------------Declare Servo Variables-----------------//

    double closedLeft = 0.03;
    double closedRight = 0.145;
    double openLeft = 0.175;
    double openRight = 0;
    double groundClaw = 0.1175;
    double scoringClaw = 0.69;

    //---------------Run OpMode-----------------------------//
    @Override
    public void runOpMode() {
        //---------------Initialize Hardware-----------------------//
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
        arm = hardwareMap.get(DcMotor.class, "arm");
        airplane = hardwareMap.get(Servo.class, "airplane");
        armROT = hardwareMap.get(DcMotor.class, "gearROT");
        clawLeft = hardwareMap.get(Servo.class, "clawleft");
        clawRight = hardwareMap.get(Servo.class, "clawright");
        clawRotate = hardwareMap.get(Servo.class, "clawrotate");
        //--------------------Setup Motors----------------------------//
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        armROT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armROT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition(0);
        armROT.setTargetPosition(13);
        //--------------------Setup Servos----------------------------//
        clawRight.setPosition(closedRight);
        clawLeft.setPosition(closedLeft);
        bspeed = 2;
        //--------------------Wait until Play----------------------------//
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        //--------------------Loop while Active----------------------------//
        while (opModeIsActive()) {

            //-------------Joysticks Controls & Wheel Power----------------//
            double max;
            double axial = -gamepad1.left_stick_y;  //Pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
            // Normalize the values so no wheel power exceeds 100%
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            //----------------------Arm Rotate & Arm Preset----------------------//

            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armROT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //This action raises the arm rotator
            if (gamepad2.right_bumper)
            {
                armRotates(1);
            }

            //This actions lowers the arm rotator
            if (gamepad2.left_bumper)
            {
                armRotates(-1);
            }

            //This actions extends the arm
            if (gamepad2.y)
            {
                armExtends(-1);
            }

            //This action retracts the arm
            if (gamepad2.a)
            {
                armExtends(1);
            }

            //This actions resets the position of the arm and arm rotator
            if (gamepad1.x)
            {
                armROT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            

            //--------------Airplane---------------//

            //This action holds the airplane servo and launches it
            if (gamepad1.y) {
                airplane.setPosition(0.5);
            } else {
                airplane.setPosition(0.1);
            }

            //---------------Claws & Claw Rotate---------------//

            //This action opens both claws
            if (gamepad2.left_trigger > 0.5)
            {
                clawRight.setPosition(openRight);
                clawLeft.setPosition(openLeft);
            }
            
            //This actions closes both claws
            if(gamepad2.right_trigger > 0.5)
            {
                clawRight.setPosition(closedRight);
                clawLeft.setPosition(closedLeft);
            }

            //This actions the left claw (when against board)
            if(gamepad2.dpad_right)
            {
                clawLeft.setPosition(openLeft);
            }

            //This action opens the right claw (when against board)
            if(gamepad2.dpad_left)
            {
                clawRight.setPosition(openRight);
            }

            //-------------------Arm-Presets--------------------//

            //This action puts the arm & claw to the ground
            if(gamepad2.x)
            {
                clawRight.setPosition(closedRight);
                clawLeft.setPosition(closedLeft);
                clawRotate.setPosition(groundClaw);
                armExtendsPosition(0,1);
                armRotatesPosition(0, .125);
            }

            //This action puts the arm & claw into scoring position
            if(gamepad2.b)
            {
                clawRight.setPosition(closedRight);
                clawLeft.setPosition(closedLeft);
                clawRotate.setPosition(scoringClaw);
                armRotatesPosition(660, 0.33);
            }
            
            //This actions puts claw to ground position
            if(gamepad2.dpad_down)
            {
                clawRotate.setPosition(groundClaw);
                armRotatesPosition(5,0.33);
            }

            //This action puts claw to scoring position
            if(gamepad2.dpad_up)
            {
                clawRotate.setPosition(scoringClaw);
            }

            //----------------Speed Control-----------------//

            //Sets the base speed to precision mode
            if (gamepad1.right_bumper) {
                bspeed = 1;
            }

            //Sets base speed to traversal mode
            if (gamepad1.left_bumper) {
                bspeed = 2;
            }

            //Sets the base speed to precision mode
            if (bspeed == 1) {
                lfSpeed = leftFrontPower / 2;
                rfSpeed = rightFrontPower / 2;
                lbSpeed = leftBackPower / 2;
                rbSpeed = rightBackPower / 2;
            }

            //Sets the base speed to precision mode
            if (bspeed == 2) {
                lfSpeed = leftFrontPower/1.2;
                rfSpeed = rightFrontPower/1.2;
                lbSpeed = leftBackPower/1.2;
                rbSpeed = rightBackPower/1.2;
            }

            leftFrontDrive.setPower(lfSpeed);
            rightFrontDrive.setPower(rfSpeed);
            leftBackDrive.setPower(lbSpeed);
            rightBackDrive.setPower(rbSpeed);

            //-------------Display Timer & Wheel Power---------------//
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }

    public void armRotatesPosition (int h, double H) {armROT.setTargetPosition(h); armROT.setPower(H); }
    
    public void armExtendsPosition (int e, double E) {arm.setTargetPosition(e); arm.setPower(E); }
    
    public void armRotates (int s) {
        armROT.setPower(0.333);
        armROT.setTargetPosition(armROT.getCurrentPosition() + 50 * s);
    }

    public void armExtends (int s) {
        arm.setPower(1);
        arm.setTargetPosition(arm.getCurrentPosition() + 150 * s);
        if (arm.getTargetPosition() > 2500) {//sets a limit on the lift distance
            arm.setTargetPosition(2500);}
    }

    public void fastArmExtends (int s) {
        arm.setPower(1);
        arm.setTargetPosition(arm.getCurrentPosition() + 250 * s);
        if (arm.getTargetPosition() > 2500) {//sets a limit on the lift distance
            arm.setTargetPosition(2500);}
    }
}