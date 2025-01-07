package org.firstinspires.ftc.teamcode.testOpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

/*
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
*/
@Disabled
@Autonomous(name="blue_left_30", group="Auto")
public class blue_left_30 extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        //MecanumDrive drive = new MecanumDrive(hardwareMap,beginPose);
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap,beginPose);

        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(3,1))
                        .build());
    }
}
