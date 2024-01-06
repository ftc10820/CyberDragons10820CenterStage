package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous
public class Qualifier1_BlueRightAutonomous extends LinearOpMode {

    public SampleMecanumDrive drive;

    // crane linear slide and lifter
    public DcMotorEx crane;
    public Servo craneAngle;

    // suspension motor
    public DcMotorEx suspension;

    // the rotating platform for intake
    public Servo indexer;

    // The servo used for intake (left and right - with the robot's perspective)
    public Servo intakeLeft;
    public Servo intakeRight;

    // the short ramp used for the intake mechanism
    public Servo ramp;

    // the drone launcher
    public Servo droneLauncher;

    // sensors used for pixel intake
    private ColorSensor pixelDetectorRight;
    private ColorSensor pixelDetectorLeft;

    // distance sensor on the bucket to detect when bucket is close to backdrop
    private DistanceSensor distanceBucket;

    // touch sensor used with linear slide
    public TouchSensor touchCrane;
    // Color sensor at the back of the robot used for detecting lines on the field
    public ColorSensor colorFieldLine;

    // Vision portal and vision processing pipelines
    private TfodProcessor tfod;
    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    int zone = 1;

    VisionSubsystem vision;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        closeLeftIntake();

        vision = new VisionSubsystem(hardwareMap);
        vision.setAlliance("blue");

        drive = new SampleMecanumDrive(hardwareMap);

        //Starting the robot at the bottom left (blue auto)
        Pose2d startPose = new Pose2d(-36, 60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        Trajectory zone1_traj1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-42,32, Math.toRadians(180)))
                .build();

        Trajectory zone1_traj2 = drive.trajectoryBuilder(zone1_traj1.end())
                .back(22.0)
                .build();

        Trajectory zone2_traj1 = drive.trajectoryBuilder(startPose)
                .back(44.0)
                .build();

        Trajectory zone3 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-42,32, Math.toRadians(180)))
                .build();

        Trajectory backstage = drive.trajectoryBuilder(zone1_traj2.end())
                .lineToLinearHeading(new Pose2d(45, 32, Math.toRadians(180)))
                .build();

        Trajectory zone2_traj2 = drive.trajectoryBuilder(zone2_traj1.end())
                .back(12.0)
                .build();

        Trajectory zone2_traj3 = drive.trajectoryBuilder(zone2_traj2.end())
                .lineToLinearHeading(new Pose2d(45,20, Math.toRadians(180)))
                .build();


        while (!isStarted()) {
            vision.elementDetection(telemetry);
            vision.returnDistance(telemetry);


            telemetry.update();

        }

        zone = vision.zone;

        waitForStart();
        if (opModeIsActive()) {

            if (zone == 1) {

                drive.followTrajectory(zone1_traj1);
                drive.followTrajectory(zone1_traj2);
                openLeftIntake();
                drive.followTrajectory(backstage);


            } else if (zone == 2) {

                drive.followTrajectory(zone2_traj1);
                openLeftIntake();
                drive.followTrajectory(zone2_traj2);
                drive.followTrajectory(zone2_traj3);


            } else {

                drive.followTrajectory(zone3);
                openLeftIntake();
                drive.followTrajectory(zone1_traj2);
                drive.followTrajectory(backstage);



            }

        }

    }

    public void initialize() {

        //crane motor
        crane = hardwareMap.get(DcMotorEx.class, "Crane");
        crane.setDirection(DcMotorEx.Direction.REVERSE);
        crane.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //suspension motor
        suspension = hardwareMap.get(DcMotorEx.class, "Suspension");
        suspension.setDirection(DcMotorEx.Direction.REVERSE);
        suspension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //servos
        indexer = hardwareMap.get(Servo.class, "Indexer");
        intakeLeft = hardwareMap.get(Servo.class, "IntakeLeft");
        intakeRight = hardwareMap.get(Servo.class, "IntakeRight");
        craneAngle = hardwareMap.get(Servo.class, "CraneAngle");
        ramp = hardwareMap.get(Servo.class, "Ramp");
        droneLauncher = hardwareMap.get(Servo.class, "DroneLauncher");


        //sensors
        touchCrane = hardwareMap.get(TouchSensor.class, "Touch");
        colorFieldLine = hardwareMap.get(ColorSensor.class, "Color");

        pixelDetectorLeft = hardwareMap.get(ColorSensor.class, "PixelDetectorLeft");
        pixelDetectorRight = hardwareMap.get(ColorSensor.class, "PixelDetectorRight");

        distanceBucket = hardwareMap.get(DistanceSensor.class, "DistanceBucket");

    }

    void openLeftIntake() {
        intakeLeft.setPosition(1.0);
    }

    void closeLeftIntake() {
        intakeLeft.setPosition(0.3);
    }

    void openRightIntake() {
        intakeRight.setPosition(0.32);
    }

    void closeRightIntake() {
        intakeRight.setPosition(0.6);
    }




}
