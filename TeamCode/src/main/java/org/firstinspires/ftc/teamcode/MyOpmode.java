package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.TimeUnit;
@Autonomous(name = "Sensor: HuskyLens", group = "Sensor")
@Disabled
public class MyOpmode extends LinearOpMode {
    //excess servos/motors
    private DcMotorEx armMotor = null; //Used to control the arm's up and down movement
    private DcMotorEx windMotor = null; //Used to control the arm's in and out movement
    private Servo clawUD = null; //Used to control the servo's up and down position
    private Servo clawLeft;
    private Servo clawRight;

    private PIDController armController;
    private PIDController windController;

    public static double ap = 0.002, ai = 0, ad = 0.0001;

    public static double wf = 0.02;

    public static double wp = 0.07, wi = 0, wd = 0.00001;

    public static int windTarget = 0;

    private final int READ_PERIOD = 1;

    private HuskyLens huskyLens;



    @Override
    public void runOpMode() {

        int globalX;
        int globalY;



        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the Driver Station telemetry.  Typical applications
         * would not likely rate limit.
         */
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        /*
         * Immediately expire so that the first time through we'll do the read.
         */
        rateLimit.expire();

        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration HuskyLens.Algorithm.
         */
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        telemetry.update();
        waitForStart();

        /*
         * Looking for AprilTags per the call to selectAlgorithm() above.  A handy grid
         * for testing may be found at https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#target_20.
         *
         * Note again that the device only recognizes the 36h11 family of tags out of the box.
         */
        while (opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            /*
             * All algorithms, except for LINE_TRACKING, return a list of Blocks where a
             * Block represents the outline of a recognized object along with its ID number.
             * ID numbers allow you to identify what the device saw.  See the HuskyLens documentation
             * referenced in the header comment above for more information on IDs and how to
             * assign them to objects.
             *
             * Returns an empty array if no objects are seen.
             */
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            String abc;
            for (int i = 0; i < blocks.length; i++) {
                int colorXValue = blocks[i].x;
                int colorYValue = blocks[i].y;
                telemetry.addData("This color x:", colorXValue);
                telemetry.addData("This color y:", colorYValue);
                telemetry.addData("Block", blocks[i].toString());
//                colorYValue = globalX;
//                colorXValue = globalY;



            }

            telemetry.update();

            //hardware mapping
//        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
//        windMotor = hardwareMap.get(DcMotorEx.class, "wind");
//        clawUD = hardwareMap.get(Servo.class, "clawUD");
//        clawLeft = hardwareMap.get(Servo.class,"clawLeft");
//        clawRight = hardwareMap.get(Servo.class,"clawRight");
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            windMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            windMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            windMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            armController = new PIDController(ap, ai, ad);
            windController = new PIDController(wp, wi, wd);

            //starting position of robot
            Pose2d startPose = new Pose2d(11, 61, Math.toRadians(-90));

            Trajectory traj1 = drive.trajectoryBuilder(startPose)
                    .splineToConstantHeading(new Vector2d(22, 37), Math.toRadians(0))
                    .build();

            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                    .splineTo(new Vector2d(45, 29), Math.toRadians(0))
                    .build();


            waitForStart();

            if (isStopRequested()) return;

            drive.followTrajectory(traj1);


        }
    }
}