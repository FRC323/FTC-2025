package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TestAuto2.Elevator.ElevatorUp;

//@Override
@Autonomous(name="TestAuto2", group="Autonomous")
public class TestAuto2 extends LinearOpMode {


    public class Elevator {

        private DcMotorEx elevator1 = null;
        private DcMotorEx elevator2 = null;

        public Elevator(HardwareMap hardwareMap) {
            elevator1 = hardwareMap.get(DcMotorEx.class, "left_elevator");
            elevator2 = hardwareMap.get(DcMotorEx.class, "right_elevator");
            elevator2.setDirection(DcMotor.Direction.REVERSE);
            elevator1.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            elevator2.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            elevator1.setMode(RunMode.RUN_USING_ENCODER);
            elevator2.setMode(RunMode.RUN_USING_ENCODER);
            elevator1.setMode(RunMode.STOP_AND_RESET_ENCODER);
            elevator2.setMode(RunMode.STOP_AND_RESET_ENCODER);
        }

        public class ElevatorUp implements Action {
            private boolean initialized = false;
            int position = 200;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    elevator1.setVelocityPIDFCoefficients(1.64, 0.0, 0, 16.4);
                    elevator2.setVelocityPIDFCoefficients(1.64, 0.0, 0, 16.4);
                    elevator1.setPositionPIDFCoefficients(15);
                    elevator2.setPositionPIDFCoefficients(15);
                    elevator1.setTargetPositionTolerance(15);
                    elevator2.setTargetPositionTolerance(15);
                }
                initialized = true;
                elevator1.setTargetPosition(position);
                elevator2.setTargetPosition(position);
                elevator1.setMode(RunMode.RUN_TO_POSITION);
                elevator2.setMode(RunMode.RUN_TO_POSITION);
                elevator1.setPower(1.0);
                elevator2.setPower(1.0);

                return false;

            }
        }

        public Action ElevatorUp() {
            return new ElevatorUp();
        }
    }
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(9, 63.25, Math.toRadians(0));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);


        // Claw claw = new Claw(hardwareMap);
        //Elevator elevator1 = new Elevator(hardwareMap);
        //Elevator elevator2 = new Elevator(hardwareMap);

        // vision here that outputs position
        //int visionOutputPosition = 1;
        waitForStart();

        //Elevator elevator;
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(9, 63.25, Math.toRadians(0)))
                        .setTangent(Math.toRadians(0))
                        .lineToX(37)
                        .waitSeconds(2)
                        .strafeTo(new Vector2d(28, 43))
                        .strafeTo(new Vector2d(63, 27))
                        .turn(Math.toRadians(180))
                        .lineToX(18)
                        .strafeTo(new Vector2d(37, 63.25))
                        .build());
        //Actions.runBlocking(new SequentialAction(Elevator.ElevatorUp()));

    }


}













