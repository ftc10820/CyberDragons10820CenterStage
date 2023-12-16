package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous

public class ScrimmageAutonomousRedRightSimple extends LinearOpMode  {

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public DcMotorEx crane;
    public Servo indexer;
    public Servo intakeLeft;
    public Servo intakeRight;
    public Servo craneAngle;
    public Servo ramp;

    public ColorSensor colorSensor;
    public TouchSensor touchSensor;

    public AprilTagProcessor aprilTag;

    public VisionPortal visionPortal;

    int zone = 1;

    VisionSubsystem vision;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        vision = new VisionSubsystem(hardwareMap);
        vision.setAlliance("blue");


        while (!isStarted()) {
            vision.elementDetection(telemetry);
            vision.returnDistance(telemetry);


            telemetry.update();

        }
        zone = vision.zone;


        //waitForStart();

        if(opModeIsActive()) {
            moveLeft(2000, 0.75);
            stopAllWheels();

            intakeLeft.setPosition(1);
            sleep(2000);

            intakeRight.setPosition(0);
            sleep(2000);

            moveBackward(100, 0.75);
        }

    }

    public void initialize() {


        // setting up drive train
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "BackRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //crane motor
        crane = hardwareMap.get(DcMotorEx.class, "Crane");

        //servos
        indexer = hardwareMap.get(Servo.class, "Indexer");
        intakeLeft = hardwareMap.get(Servo.class, "IntakeLeft");
        intakeRight = hardwareMap.get(Servo.class, "IntakeRight");
        craneAngle = hardwareMap.get(Servo.class, "CraneAngle");
        ramp = hardwareMap.get(Servo.class, "Ramp");


        ramp.setPosition(0.3);
        indexer.setPosition(0.25);
        intakeLeft.setPosition(0.3);
        intakeRight.setPosition(0.1);


        touchSensor = hardwareMap.get(TouchSensor.class, "Touch");
        colorSensor = hardwareMap.get(ColorSensor.class, "Color");



    }


    // time + speed are parameters for all the movement
    void moveBackward(int time, double speed) throws InterruptedException {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        Thread.sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    void moveBackward(double speed) throws InterruptedException {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

    }

    void moveForward(int time, double speed) throws InterruptedException {
        frontLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(-speed);

        Thread.sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    void moveForward(double speed) throws InterruptedException {
        frontLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(-speed);

    }

    void moveLeft(int time, double speed) throws InterruptedException {
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);

        Thread.sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    void moveRight(int time, double speed) throws InterruptedException {
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);

        Thread.sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }


    void turnRight(int time, double speed) throws InterruptedException {
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);

        Thread.sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    void turnLeft(int time, double speed) throws InterruptedException {
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);

        Thread.sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    void stopAllWheels() throws InterruptedException {

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
        //Changes resolution (CyberDragons Change)
        aprilTag.setDecimation(1);
        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }
    private double getDistanceToAprilTag(int zone, boolean findX){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if(detection.id == zone){
                if(findX) {
                    return detection.ftcPose.x;
                }
                else {
                    return detection.ftcPose.y;
                }
            }
        }
        return 0.0;
    }
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }
}
