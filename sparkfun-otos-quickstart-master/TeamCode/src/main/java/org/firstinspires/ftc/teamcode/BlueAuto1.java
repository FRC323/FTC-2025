package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name="BlueAuto1", group="Autonomous")
public class BlueAuto1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        // MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);

        waitForStart();
        Actions.runBlocking(drive.actionBuilder(initialPose)
                .lineToXSplineHeading(20, Math.toRadians(200))
                .waitSeconds(1)
                .lineToX(30)
                .waitSeconds(1)
                .build());

        }

    }
