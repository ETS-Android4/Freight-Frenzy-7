package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp (name= "PushButton", group= "Practice")


public class PushButtonPractice extends OpMode {

    private DcMotor intake = null;
    private Servo DuckArm = null;
    double servoPosition = 0.0;

    @Override
    public void init (){
        intake = hardwareMap.get(DcMotor.class, "intake");
        DuckArm = hardwareMap.get(Servo.class, "duckServo");
        DuckArm.setPosition(servoPosition);


    }
    @Override
    public void init_loop(){

        telemetry.addData("Practice", "Buttons");
        telemetry.update();


        //intake.setPower(0.5);
        //intake.setDirection(DcMotorSimple.Direction.FORWARD);

      if   (gamepad1.b) {
          servoPosition = 0.5;
          DuckArm.setPosition(servoPosition);
        }
      if (gamepad1.a) {
          servoPosition = 0.0;
          DuckArm.setPosition(servoPosition);

      }

    }
    @Override
    public void start() {

            intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        intake.setPower(-gamepad1.left_stick_y);
        if   (gamepad1.b) {
            servoPosition = 0.5;
            DuckArm.setPosition(servoPosition);
        }
        if (gamepad1.a) {
            servoPosition = 0.0;
            DuckArm.setPosition(servoPosition);

        }
    }

    @Override
    public void stop() {
    }



}
