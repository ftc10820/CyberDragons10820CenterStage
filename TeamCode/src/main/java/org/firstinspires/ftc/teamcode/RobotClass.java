package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

public class RobotClass {

    // drivetrain
    //public DcMotorEx frontLeft;
   // public DcMotorEx backLeft;
    //public DcMotorEx frontRight;
    //public DcMotorEx backRight;

    // vision object
    public VisionPortal camera;
    public AprilTagProcessor aprilTag;
    public TfodProcessor tfod;


    // dead wheel encoders
  //  public DcMotorEx leftEncoder;
   // public DcMotorEx rightEncoder;
   // public DcMotorEx middleEncoder;

    // field position variables
    double x;
    double y;

    public void initialize(HardwareMap hwmap) {

        //drive motor initialization
     //   frontLeft = hwmap.get(DcMotorEx.class, "FrontLeft");
      //  frontRight = hwmap.get(DcMotorEx.class, "FrontRight");
      //  backLeft = hwmap.get(DcMotorEx.class, "BackLeft");
      //  backRight = hwmap.get(DcMotorEx.class, "BackRight");

        //encoder initialization
     //   leftEncoder = hwmap.get(DcMotorEx.class, "LeftEncoder");
      //  rightEncoder = hwmap.get(DcMotorEx.class, "RightEncoder");
      //  middleEncoder = hwmap.get(DcMotorEx.class, "MiddleEncoder");

        // vision and processor initialization
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        tfod = new TfodProcessor.Builder()
                .build();

        camera = new VisionPortal.Builder()
                .setCamera(hwmap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(tfod, aprilTag)
                .build();

        camera.setProcessorEnabled(aprilTag, false);
        camera.setProcessorEnabled(tfod, false);

    }

    public double[] visionCoordinates() {

        double[] coordinates = new double[2];

        camera.setProcessorEnabled(aprilTag, true);
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            coordinates[0] = detection.ftcPose.x;
            coordinates[1] = detection.ftcPose.y;
        }
        camera.setProcessorEnabled(aprilTag, false);

        return coordinates;
    }

}
