package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    public DcMotor Lift;
    HardwareMap hwMap = null;

    /* Constructor */
    public Lift(){

    }
    /* Initialize Hardware interfaces */

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        Lift  = ahwMap.get(DcMotor.class, "Lift");

        Lift.setDirection(DcMotorSimple.Direction.FORWARD);

        Lift.setPower(0);
    }

    public void ManualLift() {
        //double LiftControl = -gamepad2.left_stick_y;
    }

}


