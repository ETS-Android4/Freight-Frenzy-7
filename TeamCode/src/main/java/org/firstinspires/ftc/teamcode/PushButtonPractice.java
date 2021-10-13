package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled

public class PushButtonPractice extends LinearOpMode {

    private DcMotor intake = null;
    private Servo duckServo = null;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Practice", "Buttons");
        telemetry.update();

        intake = hardwareMap.get(DcMotor.class, "Intake");
        duckServo = hardwareMap.get(Servo.class, "DuckServo");

        intake.setPower(double);
        intake.setDirection();
        duckServo.setPosition(0);


        waitForStart();
        runtime.reset();
    }
}
