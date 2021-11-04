package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    public boolean dForward;
    public boolean dBackward;
    public DcMotor intake;
    HardwareMap hwMap = null;

    //private variables
    private final double inSpeed = 0.555;
    private final double outSpeed = 0.555;

    /* Constructor */
    public Intake() {

    }
    /* Initialize Hardware interfaces */

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        intake = ahwMap.get(DcMotor.class, "intake");


        intake.setPower(0);
    }
//banana
    public void intake() {
        if (dForward) {
            intake.setDirection(DcMotorSimple.Direction.FORWARD);

            intake.setPower(inSpeed);
        }
        if (dBackward) {
            intake.setDirection(DcMotorSimple.Direction.REVERSE);

            intake.setPower(outSpeed);
        }
    }
}
