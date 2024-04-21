package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem {


    public static class claw {
        private Servo clawrotate = null;
        private Servo clawleft = null;
        private Servo clawright = null;
        double closedLeft = 0.03;
        double closedRight = 0.145;
        double openLeft = 0.175;
        double openRight = 0;
        double groundClaw = 0.1175;
        double scoringClaw = 0.7;

        public claw(HardwareMap hardwareMap) {
            clawrotate = hardwareMap.get(Servo.class, "clawrotate");
            clawleft = hardwareMap.get(Servo.class, "clawleft");
            clawright = hardwareMap.get(Servo.class, "clawright");
        }

        //------------------------------Close Claws------------------------------//
        public class closeLClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawleft.setPosition(closedLeft);
                return false;
            }
        }
        public Action closeLClaw() {
            return new closeLClaw();
        }
        public class closeRClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawright.setPosition(closedRight);
                return false;
            }
        }
        public Action closeRClaw() {
            return new closeRClaw();
        }
        public class closeClaws implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawleft.setPosition(closedLeft);
                clawright.setPosition(closedRight);
                return false;
            }
        }
        public Action closeClaws() {
            return new closeClaws();
        }
        //------------------------------Open Claws------------------------------//
        public class openLClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawleft.setPosition(openLeft);
                return false;
            }
        }
        public Action openLClaw() {
            return new openLClaw();
        }

        public class openRClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawright.setPosition(openRight);
                return false;
            }
        }
        public Action openRClaw() {
            return new openRClaw();
        }

        public class openClaws implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawleft.setPosition(openLeft);
                clawright.setPosition(openRight);
                return false;
            }
        }
        public Action openClaws() {
            return new openClaws();
        }

        //------------------------------Claw Rotate------------------------------//
        
        public class groundClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawrotate.setPosition(groundClaw);
                return false;
            }
        }
        public Action groundClaw() {
            return new groundClaw();
        }

        public class scoringClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawrotate.setPosition(scoringClaw);
                return false;
            }
        }
        public Action scoringClaw() {
            return new scoringClaw();
        }
    }
}
