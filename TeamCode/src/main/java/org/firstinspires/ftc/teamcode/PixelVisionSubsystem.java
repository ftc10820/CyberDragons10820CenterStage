package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class PixelVisionSubsystem {
    OpenCvCamera camera;
    DetectionPipeline detectionPipeline;
    int camW = 800;
    int camH = 448;

    int zone = 1;

    public PixelVisionSubsystem(HardwareMap hardwareMap){
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        detectionPipeline = new DetectionPipeline();

        camera.setPipeline(detectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(camW, camH, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    public void setAlliance(String alliance){
        detectionPipeline.setAlliancePipe(alliance);
    }

    public int elementDetection(Telemetry telemetry) {
        zone = detectionPipeline.get_element_zone();
        telemetry.addData("Element Zone:", zone);
        return zone;
    }

    public void returnDistance(Telemetry telemetry){
        telemetry.addData("zone 1: ", detectionPipeline.avgColor1);
        telemetry.addData("zone 2: ", detectionPipeline.avgColor2);
        telemetry.addData("zone 3: ", detectionPipeline.avgColor3);
    }

    public void toggleAverageZone(){
        detectionPipeline.toggleAverageZonePipe();
    }

}

