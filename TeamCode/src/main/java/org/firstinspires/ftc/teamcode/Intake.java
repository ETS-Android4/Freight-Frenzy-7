package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    public boolean PickUp;
    public boolean Drop;
    public DcMotor intake;
    HardwareMap hwMap = null;
    public boolean stopIntake;

    //private variables
    //banana
    private final double inPower = 1;
    private final double outPower = .65;

    /* Constructor */
    public Intake() {

    }
    /* Initialize Hardware interfaces */

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        intake = ahwMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);


        intake.setPower(0);
    }
//banana
    public void intake() {
        if (PickUp) {

            intake.setPower(inPower);
        }
        if (Drop) {

            intake.setPower(outPower);
        }
        if (stopIntake) {
            intake.setPower(0);
        }
    }
}
