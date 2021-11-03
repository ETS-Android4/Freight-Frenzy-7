package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    public boolean dForward;
    public boolean dBackward;
    public DcMotor intake;
    HardwareMap hwMap = null;

    /* Constructor */
    public Intake(){

    }
    /* Initialize Hardware interfaces */

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        intake  = ahwMap.get(DcMotor.class, "intake");

        dForward = intake.setDirection(DcMotorSimple.Direction.FORWARD);
        dBackward = intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setPower(0);
    }

    public void intake(){
        boolean dForward;
        boolean dBackward;
}

}
