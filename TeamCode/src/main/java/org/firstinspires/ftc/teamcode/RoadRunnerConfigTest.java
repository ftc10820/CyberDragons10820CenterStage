package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class RoadRunnerConfigTest extends LinearOpMode {

    VisionSubsystem vision;

    @Override
    public void runOpMode() throws InterruptedException {

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotorEx frontLeft;
        DcMotorEx frontRight;
        DcMotorEx backLeft;
        DcMotorEx backRight;

        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "BackRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");

        waitForStart();
        frontRight.setPower(.5);
        sleep(2000);
        frontRight.setPower(0.0);
        frontLeft.setPower(.5);
        sleep(2000);
        frontLeft.setPower(0.0);
        backRight.setPower(.5);
        sleep(2000);
        backRight.setPower(0.0);
        backLeft.setPower(.5);
        sleep(2000);
        backLeft.setPower(0.0);


        while(opModeIsActive()) {
            telemetry.addLine("Front Right " + frontRight.getCurrentPosition());
            telemetry.addLine("Front Left " + frontLeft.getCurrentPosition());
            telemetry.addLine("Rear Right " + backRight.getCurrentPosition());
            telemetry.addLine("Rear Left " + backLeft.getCurrentPosition());
            telemetry.update();
            if (isStopRequested()) return;
        }


    }
}