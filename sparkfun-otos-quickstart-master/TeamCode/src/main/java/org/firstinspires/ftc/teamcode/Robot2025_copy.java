/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="2025 Robot_copy", group="Iterative Opmode")
@Disabled
public class Robot2025_copy extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotorEx elevator1 = null;
    private DcMotorEx elevator2 = null;
    //private DcMotorEx elevator3 = null;
    private DcMotorEx armMotor1 = null;
    private DcMotorEx armMotor2 = null;
    
    private Servo wristRotate1 = null;
    private Servo wristRotate2 = null;
    private Servo wristTwist = null;
    private Servo claw = null;

    private DigitalChannel elevatorHome = null;
    private DigitalChannel armHome = null;
    
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    //ColorSensor leftColor;
    //ColorSensor rightColor;
    //DistanceSensor leftDistance;
    //DistanceSensor rightDistance;
    
    boolean upPulse = false;
    boolean dnPulse = false;
    boolean upIndex = false;
    boolean dnIndex = false;
    boolean eleHomeLatch;
    boolean eleZeroLatch;
    boolean initWrist = false;
    boolean initArmLatch = false;
    int wristHome;
    int wristEncoder = 0;
    int lastWristEncoder = 0;
    int pickupSequence = 0;
    int releaseSequence = 0;
    int elevatorHeight = 0;
    int elevatorIndex = 0;
    int[] elevatorSetpoints = {0,200,300,650,900,1120,1500,1400,1400,1400,1400,1400,1400};
    double maxVelocity = 0;
    int capIndex = 0;
    int capReleaseIndex = 0;
    boolean firstScan = false;
    
    //Arm Servo Positions
    double servoCenter = .435;   //Old=.465  Rev=.4305
    double servoLeft = .1;   //Old=.058  Rev=.12  .105
    double servoRight = .78;   //Old=.82  Rev=.83    .79
    
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); 

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear  = hardwareMap.get(DcMotor.class, "leftBack");
        rightRear = hardwareMap.get(DcMotor.class, "rightBack");
        
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        
        // Elevator
        elevator1 = hardwareMap.get(DcMotorEx.class, "left_elevator");
        elevator2 = hardwareMap.get(DcMotorEx.class, "right_elevator");
        //elevator3 = hardwareMap.get(DcMotorEx.class, "elevator3");
        
        elevator2.setDirection(DcMotor.Direction.REVERSE);
        
        elevator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //elevator3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        elevator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //elevator3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //elevator3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Wrist Motor 
        armMotor1 = hardwareMap.get(DcMotorEx.class, "arm_motor1");
        armMotor2 = hardwareMap.get(DcMotorEx.class, "arm_motor2");
        armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        armMotor1.setDirection(DcMotor.Direction.REVERSE);
        
        
        //Servos
        wristRotate1 = hardwareMap.servo.get("wristRotate1");
        wristRotate2 = hardwareMap.servo.get("wristRotate2");
        wristTwist = hardwareMap.servo.get("wristTwist");
        claw = hardwareMap.servo.get("claw");
        
        wristRotate2.setDirection(Servo.Direction.REVERSE);
        //armRotateTop.setDirection(Servo.Direction.REVERSE);
        
        
        elevatorHome = hardwareMap.digitalChannel.get("elevatorHome");
        armHome = hardwareMap.digitalChannel.get("armHome");
        
        
        
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        //imu.initialize(parameters);
      
        // get a reference to the color and Distance sensor.
        //leftColor = hardwareMap.get(ColorSensor.class, "leftColor");
        //rightColor = hardwareMap.get(ColorSensor.class, "rightColor");
        //leftDistance = hardwareMap.get(DistanceSensor.class, "leftColor");
        //rightDistance = hardwareMap.get(DistanceSensor.class, "rightColor");
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;
      
      
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        
     //imu.initialize(parameters);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        
        runtime.reset();
        elevator1.setVelocityPIDFCoefficients(1.64, 0.0, 0, 16.4);
        elevator2.setVelocityPIDFCoefficients(1.64, 0.0, 0, 16.4);
        elevator1.setPositionPIDFCoefficients(15);
        elevator2.setPositionPIDFCoefficients(15);
        elevator1.setTargetPositionTolerance(15);
        elevator2.setTargetPositionTolerance(15);

        armMotor1.setVelocityPIDFCoefficients(1.00, 0.0, 0, 16.0);
        armMotor2.setVelocityPIDFCoefficients(1.00, 0.0, 0, 16.0);
        armMotor1.setPositionPIDFCoefficients(15);
        armMotor2.setPositionPIDFCoefficients(15);
        armMotor1.setTargetPositionTolerance(20);
        armMotor2.setTargetPositionTolerance(20);
        
        // Set starting positions for arm and pickup
        wristRotate1.setPosition(servoCenter);
        wristRotate2.setPosition(servoCenter);
        wristRotate1.setPosition(.3 + .03);
        wristRotate2.setPosition(.3);
        wristTwist.setPosition(1);
        claw.setPosition(.3);
        firstScan = true;


    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    
     
    public void loop() {
        
        double pickupStick = 0;
        // Setup a variable for each drive wheel to save power level for telemetry
        double wheelSpeeds[] = new double [4];
    
 
        //----------------------- Read Joysticks ----------------------
        double x = gamepad1.right_stick_x;
        double y = gamepad1.right_stick_y;
        double rotation = gamepad1.left_stick_x;
        
        //---------------------- Non-Linearize Joysticks -------------
        
        if(gamepad1.right_bumper)
        {
        x = x * .68;
        y = y * .68;
        rotation = rotation * .55;
        }
        x = .8*Math.pow(x,3)+(1-.8)*x;
        y = .8*Math.pow(y,3)+(1-.8)*y;
        rotation = (.7*Math.pow(rotation,3)+(1-.7)*rotation);
        
        //---------------------- Field-Centric Calculations ----------- 
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double gyro_radians = angles.firstAngle *(3.14159/180);
        double temp;
        //if(gamepad1.left_trigger < .5)
        //{
        //temp = y * Math.cos(gyro_radians) + x * Math.sin(gyro_radians);
        //x = -y * Math.sin(gyro_radians) + x * Math.cos(gyro_radians);
        //y = temp;
        //}
    
        wheelSpeeds[0] = -x + y - rotation;
        wheelSpeeds[3] = x + y + rotation;
        wheelSpeeds[1] = x + y - rotation;
        wheelSpeeds[2] = -x + y + rotation;
        
        normalize(wheelSpeeds);
        
        // Send calculated power to wheels
        leftFront.setPower(wheelSpeeds[0]);
        rightFront.setPower(wheelSpeeds[3]);
        leftRear.setPower(wheelSpeeds[1]);
        rightRear.setPower(wheelSpeeds[2]);
        
        //--------------------Rezero Gyro--------------------------
        if(gamepad1.back)
            imu.initialize(parameters);    
        
        
        //---------------------Pickup Gamepiece----------------------------


        //double wrist = gamepad2.right_stick_y;
        //wrist = (wrist/2) +.5;

        if(gamepad1.right_bumper && pickupSequence <= 50) {
            if (pickupSequence > -1 && pickupSequence < 20) {
                wristRotate1.setPosition(.3 + .03);
                wristRotate2.setPosition(.3);
                wristTwist.setPosition(1);
                claw.setPosition(.5);
                pickupSequence++;
            }
            if (pickupSequence > 19 && pickupSequence <= 50) {
                wristRotate1.setPosition(.48 + .03);
                wristRotate2.setPosition(.48);
                claw.setPosition(.28);
                pickupSequence++;
            }
        }
            if (!gamepad1.right_bumper && pickupSequence > 20) {
                pickupSequence = 0;
                wristRotate1.setPosition(.3 + .03);
                wristRotate2.setPosition(.3);
                wristTwist.setPosition(1);
                claw.setPosition(.28);
            }

//----------------------------Release Gamepiece---------------------------------

            if (gamepad1.left_bumper && releaseSequence <= 40) {
                if (releaseSequence > -1 && releaseSequence < 30) {
                    wristRotate1.setPosition(.3 + .03);
                    wristRotate2.setPosition(.3);
                    wristTwist.setPosition(0);
                    claw.setPosition(.28);
                    releaseSequence++;
                }
                if (releaseSequence > 29 && releaseSequence <= 40) {
                    wristRotate1.setPosition(.48 + .03);
                    wristRotate2.setPosition(.48);
                    claw.setPosition(.5);
                    releaseSequence++;
                }
            }

            if (!gamepad1.left_bumper && releaseSequence > 39) {
                releaseSequence = 0;
                wristRotate1.setPosition(.48 + .03);
                wristRotate2.setPosition(.48);
            wristTwist.setPosition(1);
            claw.setPosition(.28);
        }




        /*       if (gamepad1.x || gamepad2.x)
            {
            armRotateBot.setPosition(servoLeft);
            armRotateTop.setPosition(servoLeft);
            if(wristHome == 0)
                {
                wristMotor.setTargetPosition(15);
                wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wristMotor.setPower(0.80);
                }
            }
        if(gamepad1.y || gamepad2.y)
            {
            armRotateBot.setPosition(servoCenter);
            armRotateTop.setPosition(servoCenter);
            if(wristHome == 0)
                {
                wristMotor.setTargetPosition(15);
                wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wristMotor.setPower(0.80);
                }
            }
        if(gamepad1.b || gamepad2.b)
            {
            armRotateBot.setPosition(servoRight);
            armRotateTop.setPosition(servoRight);
            if(wristHome > 0)
                {
                wristMotor.setTargetPosition(15);
                wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wristMotor.setPower(0.80);
                }
            }
*/
        //----------------------Arm Rotation---------------------------
        double ArmUp =gamepad2.left_trigger;
        double ArmUp2 =gamepad2.right_trigger;

         if(!initArmLatch) {
            if(!gamepad2.left_bumper && !gamepad2.right_bumper && (ArmUp < .5) && ArmUp2 < .5 )
            {
                double armMotor = -gamepad2.left_stick_y;
                armMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armMotor1.setPower(armMotor);
                armMotor2.setPower(armMotor);
            }

        if(gamepad2.left_bumper)
            {
            armMotor1.setTargetPosition(-50);
            armMotor2.setTargetPosition(-50);
            armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor1.setPower(.35);
            armMotor2.setPower(.35);
            }

         else if(gamepad2.right_bumper)
             {
                 armMotor1.setTargetPosition(1100);
                 armMotor2.setTargetPosition(1100);
                 armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 armMotor1.setPower(.35);
                 armMotor2.setPower(.35);
             }

         else if(ArmUp > .5 || ArmUp2 > .5)
            {
                armMotor1.setTargetPosition(0);
                armMotor2.setTargetPosition(0);
                armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor1.setPower(.45);
                armMotor2.setPower(.45);
            }
         }
         //idk if i should place these here -m
         Pose2d beginPose = new Pose2d(0, 0, 0);
         SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap,beginPose);

             //Auto code
         Actions.runBlocking(
                 drive.actionBuilder(beginPose)
                         .strafeTo(new Vector2d(30, 0))
                         .build());

        //if(gamepad2.dpad_left)
        //{
        //    wristMotor.setTargetPosition(130);
        //    wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //    wristMotor.setPower(0.90);
        //}
        
        //if(gamepad2.dpad_right)
        //{
        //    wristMotor.setTargetPosition(10);
        //    wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //    wristMotor.setPower(0.70);
        //}
        
        
/*
        //-----------------Release Stone Sequence-------------------
        
        double leftStoneDistance = leftDistance.getDistance(DistanceUnit.CM);
        double rightStoneDistance = rightDistance.getDistance(DistanceUnit.CM);
        
        if(gamepad1.left_bumper )
        {
            pickupSequence = 0;
            if(releaseSequence > -1 && releaseSequence < 8)
                {
                //rightLift.setPosition(0.45);
                //leftLift.setPosition(0.45);
                wristMotor.setTargetPosition(110);
                wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wristMotor.setPower(0.70);
                releaseSequence = releaseSequence + 1;
                }
        }  
        if(!gamepad1.left_bumper && releaseSequence > 7 )
        {
            if (releaseSequence > 7 && releaseSequence < 15)
                {
                wristMotor.setTargetPosition(125);
                wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wristMotor.setPower(0.70);
                releaseSequence++;
                }
            if (releaseSequence > 14 && releaseSequence < 25)
                {
                stoneLatch.setPosition(0.28);
                releaseSequence++;
                }
            if (releaseSequence > 24 && releaseSequence < 28)
                {
                wristMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                wristMotor.setPower(-.6);
                releaseSequence++;
                }    
            if (releaseSequence > 27 && releaseSequence < 35)
                {
                wristEncoder = wristMotor.getCurrentPosition();
                if (lastWristEncoder >= wristEncoder - 1 && lastWristEncoder <= wristEncoder + 1)
                releaseSequence++;
                lastWristEncoder = wristEncoder;
                }
            if(releaseSequence > 34 && releaseSequence < 38)    
                { 
                wristMotor.setTargetPosition(0);
                wristMotor.setPower(0.0);
                wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                wristMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                releaseSequence++;
                }
            if(releaseSequence > 37 && releaseSequence <= 39)
                {
                wristMotor.setTargetPosition(15);
                wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wristMotor.setPower(0.80);
                releaseSequence++;
                }
            if(releaseSequence == 39)
                {
                releaseSequence = 0;
                pickupSequence = 0;
                }
         }   
        
         //-----------------Pickup Stone Sequence-------------------    
        if(gamepad1.left_trigger > .5)
        {   
            if(pickupSequence == 0)
                {
                wristMotor.setTargetPosition(122);
                wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wristMotor.setPower(0.90);
                pickupSequence = 1;
                releaseSequence = 0;
                double ccR = (rightColor.red() / rightColor.blue()) * (rightColor.green() / rightColor.blue());

                }
            if((leftStoneDistance < 2.0 && rightStoneDistance < 2.0) && pickupSequence > 0 && pickupSequence < 12)   
                { 
                //rightLift.setPosition(0.52);
                //leftLift.setPosition(0.52);
                wristMotor.setTargetPosition(132);
                wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wristMotor.setPower(1.0);
                if(pickupSequence > 3)
                stoneLatch.setPosition(0-.020);
                pickupSequence = pickupSequence + 1;
                }
                
            if(pickupSequence > 11 && pickupSequence <= 35)
                {
                //rightLift.setPosition(0.15);
                //leftLift.setPosition(0.15);
                wristMotor.setTargetPosition(15);
                wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wristMotor.setPower(0.90);
                pickupSequence = pickupSequence + 1;
                }
         }
         
         if(pickupSequence > 35 && gamepad1.left_trigger < .5)
                {
                pickupSequence = 0;
                releaseSequence = 0;
                }
        

 */
        //-----------------Run Elevator----------------

        double ArmDown =gamepad2.left_trigger;
        double elevatorDn2 =gamepad2.right_trigger;
        
        if(( gamepad2.right_bumper  || gamepad2.left_bumper || firstScan) && !upPulse)
            {
            elevatorIndex++;
            if(elevatorIndex > 5)
                elevatorIndex = 5;

            if(gamepad2.left_bumper)
                    elevatorIndex = 6;
            elevatorHeight = elevatorSetpoints[elevatorIndex];
            
            elevator1.setTargetPosition(elevatorHeight);
            elevator2.setTargetPosition(elevatorHeight);
            elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator1.setPower(1.0);
            elevator2.setPower(1.0);

            upPulse = true;
            }
        else if((elevatorDn2 > .5)|| ArmDown > .5 && !dnPulse)
            {
            elevatorHeight = 200;
            elevatorIndex = 0;
            elevator1.setTargetPosition(elevatorHeight);
            elevator2.setTargetPosition(elevatorHeight);
            //elevator3.setTargetPosition(elevatorHeight);
            elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //elevator3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator1.setPower(0.7);
            elevator2.setPower(0.7);
            //elevator3.setPower(0.5);
            dnPulse = true;
            }
        
        if(!gamepad2.right_bumper && upPulse) 
            upPulse = false;
        
        if( elevatorDn2 < .4 && elevatorDn2 < .4  && dnPulse ) 
            dnPulse = false;
            
        if(gamepad2.dpad_up &! upIndex)
            {
            elevatorIndex++;
            if(elevatorIndex > 5)
                elevatorIndex =5;
            elevatorHeight = elevatorSetpoints[elevatorIndex];    
            upIndex = true;    
            }
        if (!gamepad2.dpad_up && upIndex)
            upIndex = false;
            
        if(gamepad2.dpad_down &! dnIndex)
            {
            elevatorIndex--;
            elevatorIndex--;
            if(elevatorIndex < 0)
                elevatorIndex = 0;
            elevatorHeight = elevatorSetpoints[elevatorIndex];    
            dnIndex = true;    
            }
        if(!gamepad2.dpad_down && dnIndex)
            dnIndex = false;
            
        
        //------------------------- Home elevator encoders ------------------
        
        if((gamepad2.back) &! eleHomeLatch)
            {
            eleHomeLatch = true;
            elevator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elevator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elevator1.setPower(-.45);
            elevator2.setPower(-.45);
            }
        if (eleHomeLatch && !elevatorHome.getState())
            { 
            elevator1.setTargetPosition(0);
            elevator2.setTargetPosition(0);
            elevator1.setPower(0.0);
            elevator2.setPower(0.0);
            elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elevator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elevatorIndex = 0;
            }
        if((!gamepad2.back) && eleHomeLatch && !elevatorHome.getState())
                {
                eleHomeLatch = false;
                }

        //------------------------- Home arm encoder ------------------
        //double wrist = gamepad1.right_trigger;
        if((gamepad2.start) || firstScan &! initArmLatch)
            {
            initArmLatch = true;
            armMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor1.setPower(-.35);
            armMotor2.setPower(-.35);
            }
        if (initArmLatch && !armHome.getState())
        {

            armMotor1.setPower(0.0);
            armMotor2.setPower(0.0);
            armMotor1.setTargetPosition(0);
            armMotor2.setTargetPosition(0);
            armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        if((!gamepad1.back) && initArmLatch && !armHome.getState())
        {
            initArmLatch = false;
        }

        firstScan = false;





        // Read Odometry Encoders    
        //double leftDistance = (leftFront.getCurrentPosition())  * -1;  //1350 per in.
        //double rightDistance = (rightFront.getCurrentPosition()) * -1;//1350 per in.
        //double sideDistance = (leftRear.getCurrentPosition()) * -1;    //1350 per in.
        
        //if(gamepad1.back)
        //{
        //    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //    rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //    leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //    leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //    rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //    leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //} 
        
        // **************temporary elevator tuning****************8
        //elevator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //elevator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //elevator3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //double elevatorPower = gamepad2.right_stick_y;
        //elevator1.setPower(elevatorPower);
        //elevator2.setPower(elevatorPower);
        //elevator3.setPower(elevatorPower);
        //double velocity = elevator1.getVelocity();
        //if(velocity > maxVelocity)
        //    maxVelocity = velocity;
        //DcMotorControllerEx motorControllerEx = (DcMotorControllerEx)elevator1.getController();
        //int motorIndex = ((DcMotorEx)elevator1).getPortNumber();
        //PIDFCoefficients pidOrig = motorControllerEx.getPIDFCoefficients(motorIndex,DcMotor.RunMode.RUN_USING_ENCODER);
        
        //telemetry.addData("PID", "%.04f, %.04f, %.0f,%.0f",pidOrig.p, pidOrig.i, pidOrig.d, pidOrig.f);
        //telemetry.addData("Tolerance",elevator1.getTargetPositionTolerance());
        
        // Show the elapsed game time and wheel power.
        //telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Sticks", "Y (%.2f), X (%.2f) Rot (%.2f)", y, x, rotation);
        //telemetry.addData("Heading", "(%f2.1)", angles.firstAngle);
        //telemetry.addData("Distance",
        //            String.format(Locale.US, "%.02f,  %.02f", leftStoneDistance,rightStoneDistance));
        //telemetry.addData(String.format(Locale.US,"(%0.2f),(%0.2f)", leftDistance.getDistance(DistanceUnit.CM), rightDistance.getDistance(DistanceUnit.CM)));
        //telemetry.addData("Actual",  "%7d:%7d:%7d",      elevator1.getCurrentPosition(),
        //                                                 elevator2.getCurrentPosition(),
        //                                                 elevator3.getCurrentPosition());
      //double ccL = (leftColor.red() / leftColor.blue()) * (leftColor.green() / leftColor.blue());
      //telemetry.addData("CCL",ccL);
      //double ccR = (rightColor.red() / rightColor.blue()) * (rightColor.green() / rightColor.blue());
      //telemetry.addData("CCR",ccR);
        //telemetry.addData("Blue",leftColor.blue());
        telemetry.addData("EleIndex",elevatorIndex);
        telemetry.addData("Elevator",elevator1.getCurrentPosition());
        telemetry.addData("Elevator",elevator2.getCurrentPosition());
        telemetry.addData("ElevatorIndex",elevatorIndex);
        telemetry.addData("Arm1",armMotor1.getCurrentPosition());
        telemetry.addData("Arm2",armMotor2.getCurrentPosition());
        telemetry.addData("ArmHome",armHome.getState());
        telemetry.addData("pickupSeq",pickupSequence);
        //telemetry.addData("wrist",wrist);
        //telemetry.addData("DN",dnPulse);
        //telemetry.addData("Pickup",pickupSequence);
        //telemetry.addData("Release",releaseSequence);
        //telemetry.addData("CapIndex",capIndex);
        //telemetry.addData("CapReleaseIndex",capReleaseIndex);
        //telemetry.addData("Wrist",wristMotor.getCurrentPosition());
        //telemetry.addData("WristHome",wristHome);
        
        
        //telemetry.addData("LeftOdometer",String.format(Locale.US, "%.02f", leftDistance));
        //telemetry.addData("RightOdometer",String.format(Locale.US, "%.02f", rightDistance));
        //telemetry.addData("SideOdometer",String.format(Locale.US, "%.02f", sideDistance));

    }

    private void normalize(double[] wheelSpeeds)
    {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
            for (int i = 1; i< wheelSpeeds.length; i++)
            {
                double magnitude = Math.abs(wheelSpeeds[i]);
                if (magnitude > maxMagnitude)
                {
                maxMagnitude = magnitude;
                }   
            }
            if (maxMagnitude > 1.0)
            {
                for (int i = 0; i < wheelSpeeds.length; i++)
                {
                    wheelSpeeds[i] /= maxMagnitude;
                }   
            }
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
