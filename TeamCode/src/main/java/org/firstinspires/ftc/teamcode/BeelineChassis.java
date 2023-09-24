package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class BeelineChassis extends LinearOpMode {

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public DcMotorEx intake;
    public DcMotorEx linearSlide;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();
        while (opModeIsActive()) {

            frontRight.setPower(gamepad1.right_stick_y);
            backRight.setPower(gamepad1.right_stick_y);

            frontLeft.setPower(-gamepad1.left_stick_y);
            backLeft.setPower(-gamepad1.left_stick_y);

            linearSlide.setPower(gamepad2.right_stick_y);

            if (gamepad2.y) {

                linearSlide.setPower(1);
            } else if (gamepad2.a) {

                linearSlide.setPower(-1);
            } else if (gamepad2.x) {

                linearSlide.setPower(0);

            }


        }

    }

    void initialize() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "BackRight");

        linearSlide = hardwareMap.get(DcMotorEx.class, "LinearSlide");
        intake = hardwareMap.get(DcMotorEx.class, "Intake");

    }
}
