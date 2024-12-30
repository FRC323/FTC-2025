package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "TestAuto1", group = "Autonomous")
public class TestAuto1 extends LinearOpMode {
    public class Elevator {
        private DcMotorEx elevator1;
        private DcMotorEx elevator2;


        public Elevator(HardwareMap hardwareMap) {
            elevator1 = hardwareMap.get(DcMotorEx.class, "left_elevator");
            elevator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            elevator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            elevator2 = hardwareMap.get(DcMotorEx.class, "right_elevator");
            elevator2.setDirection(DcMotor.Direction.REVERSE);
            elevator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            elevator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }

        public class ElevatorUp implements Action {
            Elevator elevator;
            int position;
            private boolean initialized = false;
            public ElevatorUp(Elevator e, int p) {
                this.elevator = e;
                this.position = p;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    elevator1.setVelocityPIDFCoefficients(1.64, 0.0, 0, 16.4);
                    elevator2.setVelocityPIDFCoefficients(1.64, 0.0, 0, 16.4);
                    elevator1.setPositionPIDFCoefficients(15);
                    elevator2.setPositionPIDFCoefficients(15);
                    elevator1.setTargetPositionTolerance(15);
                    elevator2.setTargetPositionTolerance(15);
                    initialized = true;
                }
                elevator1.setTargetPosition(400);
                elevator2.setTargetPosition(400);
                elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator1.setPower(1.0);
                elevator2.setPower(1.0);
                return elevator1.getTargetPosition() < 375;
            }
        }
/*
        public Action elevatorUp() {
            return new ElevatorUp();
        }
*/
        public class ElevatorDn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    elevator1.setVelocityPIDFCoefficients(1.64, 0.0, 0, 16.4);
                    elevator2.setVelocityPIDFCoefficients(1.64, 0.0, 0, 16.4);
                    elevator1.setPositionPIDFCoefficients(15);
                    elevator2.setPositionPIDFCoefficients(15);
                    elevator1.setTargetPositionTolerance(15);
                    elevator2.setTargetPositionTolerance(15);
                    initialized = true;
                }
                elevator1.setTargetPosition(250);
                elevator2.setTargetPosition(250);
                elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator1.setPower(-0.8);
                elevator2.setPower(-0.8);
                return elevator1.getTargetPosition() > 275;
            }
        }

        public Action elevatorDn() {
            return new ElevatorDn();
        }
    }

    public class Arm {
        private DcMotorEx armMotor1;
        private DcMotorEx armMotor2;
        private DigitalChannel armHome = null;


        public Arm(HardwareMap hardwareMap) {
            armMotor1 = hardwareMap.get(DcMotorEx.class, "arm_motor1");
            armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor1.setDirection(DcMotor.Direction.REVERSE);

            armMotor2 = hardwareMap.get(DcMotorEx.class, "arm_motor2");
            armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            armHome = hardwareMap.digitalChannel.get("armHome");

        }

        public class ArmDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor1.setVelocityPIDFCoefficients(1.00, 0.0, 0, 16.0);
                    armMotor2.setVelocityPIDFCoefficients(1.00, 0.0, 0, 16.0);
                    armMotor1.setPositionPIDFCoefficients(15);
                    armMotor2.setPositionPIDFCoefficients(15);
                    armMotor1.setTargetPositionTolerance(20);
                    armMotor2.setTargetPositionTolerance(20);
                    initialized = true;
                }
                armMotor1.setTargetPosition(1150);
                armMotor2.setTargetPosition(1150);
                armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor1.setPower(.50);
                armMotor2.setPower(.50);
                return armMotor1.getTargetPosition() < 1050;
            }
        }

        public Action armDown() {
            return new ArmDown();
        }

        public class ArmHook implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor1.setVelocityPIDFCoefficients(1.00, 0.0, 0, 16.0);
                    armMotor2.setVelocityPIDFCoefficients(1.00, 0.0, 0, 16.0);
                    armMotor1.setPositionPIDFCoefficients(15);
                    armMotor2.setPositionPIDFCoefficients(15);
                    armMotor1.setTargetPositionTolerance(20);
                    armMotor2.setTargetPositionTolerance(20);
                    initialized = true;
                }
                armMotor1.setTargetPosition(400);
                armMotor2.setTargetPosition(400);
                armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor1.setPower(.50);
                armMotor2.setPower(.50);
                return armMotor1.getTargetPosition() < 350;
            }
        }

        public Action armHook() {
            return new ArmHook();
        }

        public class ArmBack implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor1.setVelocityPIDFCoefficients(1.00, 0.0, 0, 16.0);
                    armMotor2.setVelocityPIDFCoefficients(1.00, 0.0, 0, 16.0);
                    armMotor1.setPositionPIDFCoefficients(15);
                    armMotor2.setPositionPIDFCoefficients(15);
                    armMotor1.setTargetPositionTolerance(20);
                    armMotor2.setTargetPositionTolerance(20);
                    initialized = true;
                }
                armMotor1.setTargetPosition(-100);
                armMotor2.setTargetPosition(-100);
                armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor1.setPower(-.50);
                armMotor2.setPower(-.50);
                return armMotor1.getTargetPosition() < 0;
            }
        }

        public Action armBack() {
            return new ArmBack();
        }


        // --------------------------- Arm Home ----------------------------
        public class ArmHome implements Action {
            private boolean initialized = false;
            private boolean initArmLatch = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor1.setVelocityPIDFCoefficients(1.00, 0.0, 0, 16.0);
                    armMotor2.setVelocityPIDFCoefficients(1.00, 0.0, 0, 16.0);
                    armMotor1.setPositionPIDFCoefficients(15);
                    armMotor2.setPositionPIDFCoefficients(15);
                    armMotor1.setTargetPositionTolerance(20);
                    armMotor2.setTargetPositionTolerance(20);
                    initialized = true;
                }
                if (!initArmLatch) {
                    initArmLatch = true;
                    armMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armMotor1.setPower(-.45);
                    armMotor2.setPower(-.45);
                }
                if (initArmLatch && !armHome.getState()) {
                    armMotor1.setPower(0.0);
                    armMotor2.setPower(0.0);
                    armMotor1.setTargetPosition(0);
                    armMotor2.setTargetPosition(0);
                    armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    initArmLatch = false;
                }
                    return initArmLatch;
                }

        }

        public Action armHome() {
            return new Arm.ArmHome();
        }
    }



        public class Claw {
            private Servo claw;

            public Claw(HardwareMap hardwareMap) {
                claw = hardwareMap.get(Servo.class, "claw");
            }


            public class CloseClaw implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    claw.setPosition(1);
                    return false;
                }
            }

            public Action closeClaw() {
                return new CloseClaw();
            }

            public class OpenClaw implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    claw.setPosition(0.50);
                    return false;
                }
            }

            public Action openClaw() {
                return new OpenClaw();
            }
        }

        public class Wrist {
            private Servo wristRotate1;

            public Wrist(HardwareMap hardwareMap) {
                wristRotate1 = hardwareMap.servo.get("wristRotate1");
                wristRotate1.setDirection(Servo.Direction.REVERSE);
            }


            public class WristPickupPos implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    wristRotate1.setPosition(0.48);
                    return false;
                }
            }

            public Action wristPickupPos() {
                return new WristPickupPos();
            }

            public class OpenClaw implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    wristRotate1.setPosition(1.0);
                    return false;
                }
            }

            public Action openClaw() {
                return new OpenClaw();
            }
        }

        @Override
        public void runOpMode() throws InterruptedException {
            Pose2d initialPose = new Pose2d(9, 63.25, Math.toRadians(0));
            SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
            //Claw claw = new Claw(hardwareMap);
            Elevator elevator = new Elevator(hardwareMap);
            Arm arm = new Arm(hardwareMap);
            Wrist wrist = new Wrist(hardwareMap);
            Claw claw = new Claw(hardwareMap);

            // vision here that outputs position
            int visionOutputPosition = 1;

            TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                    .setTangent(Math.toRadians(0))
                    .stopAndAdd(arm.armHome())
                    .stopAndAdd(claw.closeClaw())
                    .stopAndAdd(wrist.wristPickupPos())
                    /*psuedo for score hanging piece
                    .elevatorUp(pos)
                    .armHook()
                    .move
                    .elevatorDn(~350)
                    .openClaw()
                    .
                    */
                    //vv
                    .lineToX(37)
                    .waitSeconds(2)
                    .strafeTo(new Vector2d(28, 43))
                    .strafeTo(new Vector2d(63, 27))
                    .turn(Math.toRadians(180))
                    .lineToX(18)
                    .strafeTo(new Vector2d(37, 63.25));
            TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                    //.lineToY(37)
                    //.setTangent(Math.toRadians(0))
                    //.lineToX(18)
                    //.waitSeconds(3)
                    //.setTangent(Math.toRadians(0))
                    //.lineToXSplineHeading(46, Math.toRadians(180))
                    .waitSeconds(3);
            TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                    //.lineToYSplineHeading(33, Math.toRadians(180))
                    //.waitSeconds(2)
                    //.strafeTo(new Vector2d(46, 30))
                    .waitSeconds(3);
            Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                    .strafeTo(new Vector2d(18, 27))
                    .build();

            // actions that need to happen on init; for instance, a claw tightening.
            //Actions.runBlocking(claw.closeClaw());


            while (!isStopRequested() && !opModeIsActive()) {
                int position = visionOutputPosition;
                telemetry.addData("Position during Init", position);
                telemetry.update();
            }

            int startPosition = visionOutputPosition;
            telemetry.addData("Starting Position", startPosition);
            telemetry.update();
            waitForStart();

            if (isStopRequested()) return;

            Action trajectoryActionChosen;
            if (startPosition == 1) {
                trajectoryActionChosen = tab1.build();
            } else if (startPosition == 2) {
                trajectoryActionChosen = tab2.build();
            } else {
                trajectoryActionChosen = tab3.build();
            }

            Actions.runBlocking(
                    new SequentialAction(
                            //arm.armHome(),
                            arm.armHook(),
                            trajectoryActionChosen,
                            //claw.openClaw(),
                            //lift.liftDown(),
                            trajectoryActionCloseOut
                    )
            );
        }
    }
