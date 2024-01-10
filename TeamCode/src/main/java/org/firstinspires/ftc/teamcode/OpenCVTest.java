package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.DetectionPipeline;
import org.firstinspires.ftc.teamcode.OpenCVTest;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Autonomous
@Disabled
public class OpenCVTest extends LinearOpMode {

    VisionSubsystem vision;

    @Override
    public void runOpMode() throws InterruptedException {

        vision = new VisionSubsystem(hardwareMap);
        vision.setAlliance("blue");
        vision.returnDistance(telemetry);
        vision.elementDetection(telemetry);
        telemetry.update();


        waitForStart();
    }
}