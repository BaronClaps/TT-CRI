package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmRotationSubsystem {
    public static class armRotation {
        private DcMotorEx armROT;

        public armRotation(HardwareMap hardwareMap) {
            armROT = hardwareMap.get(DcMotorEx.class, "gearROT");
            armROT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armROT.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        
        //------------------------------Ground Position------------------------------//
        
        public class armRotationGround implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                armROT.setTargetPosition(0);
                armROT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armROT.setPower(0.125);
                return false;
            } 
        }
        
        public Action armRotationGround() {
            return new armRotationGround();
        }

        //------------------------------Scoring Position------------------------------//
        
        public class armRotationScoring implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armROT.setTargetPosition(670);
                armROT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armROT.setPower(0.4);
                return false;
            }
        }

        public Action armRotationScoring() {
            return new armRotationScoring();
        }

        //------------------------------Zero Power-------------------------------//
        
        public class armRotationZeroPower implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                armROT.setPower(0);
                return false;
            }
        }

        public Action armRotationZeroPower() {
            return new armRotationZeroPower();
        }
    }
}
