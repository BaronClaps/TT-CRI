package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PresetSubsystem {

    public static class presets {

        private DcMotorEx arm;
        private DcMotorEx armROT;
        private Servo clawrotate = null;
        private Servo clawleft = null;
        private Servo clawright = null;
        double closedLeft = 0.03;
        double closedRight = 0.145;
        double openLeft = 0.175;
        double openRight = 0;
        double groundClaw = 0.1175;
        double scoringClaw = 0.7;

        public presets(HardwareMap hardwareMap) {
            clawrotate = hardwareMap.get(Servo.class, "clawrotate");
            clawleft = hardwareMap.get(Servo.class, "clawleft");
            clawright = hardwareMap.get(Servo.class, "clawright");
            arm = hardwareMap.get(DcMotorEx.class, "arm");
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setDirection(DcMotorSimple.Direction.FORWARD);
            armROT = hardwareMap.get(DcMotorEx.class, "gearROT");
            armROT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armROT.setDirection(DcMotorSimple.Direction.FORWARD);
        }


        public class startPos implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                armROT.setTargetPosition(75);
                armROT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armROT.setPower(0.4);
                clawrotate.setPosition(groundClaw);
                return false;
            }
        }
        public Action startPos() {
            return new startPos();
        }


        public class groundPos implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                armROT.setTargetPosition(75);
                armROT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armROT.setPower(0.125);
                clawrotate.setPosition(groundClaw);
                clawleft.setPosition(closedLeft);
                clawright.setPosition(closedRight);
                return false;
            }
        }

        public Action groundPos() {
            return new groundPos();
        }



        public class scoringPos implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                armROT.setTargetPosition(670);
                armROT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armROT.setPower(0.4);
                clawrotate.setPosition(scoringClaw);
                return false;
            }
        }

        public Action scoringPos() {
            return new scoringPos();
        }

        public class resetEncoders implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                armROT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                return false;
            }
        }

        public Action resetEncoders() {
            return new resetEncoders();
        }




    }
}
