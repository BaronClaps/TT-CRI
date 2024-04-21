package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem {
    public static class arm {
        private DcMotorEx arm;

        public arm(HardwareMap hardwareMap) {
            arm = hardwareMap.get(DcMotorEx.class, "arm");
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class armExtend implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                arm.setTargetPosition(-200);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.7);
                return false;
            }
        }

        public Action armExtend() {
            return new armExtend();
        }

        public class armRetract implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                arm.setTargetPosition(200);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.7);
                return false;
            }
        }

        public Action armRetract() {
            return new armRetract();
        }

        public class armReset implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                return false;
            }
        }

        public Action armReset() {
            return new armReset();
        }
    }

}



