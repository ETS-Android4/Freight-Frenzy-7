package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

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

        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setPower(0);
    }

}
