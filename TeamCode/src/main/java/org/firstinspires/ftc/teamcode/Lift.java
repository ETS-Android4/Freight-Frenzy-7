package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {

    //public DcMotor Lift;
    public DcMotorEx Lift;
    public int liftPosition;
    HardwareMap hwMap = null;
    public double elevator = 0;

    private final double low = 2;
    private final double mid = 4;
    private final double high = 6;
    private final double mult = 537.7;


    /* Constructor */
    public Lift(){

    }
    /* Initialize Hardware interfaces */

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        Lift  = ahwMap.get(DcMotorEx.class, "Lift");

        Lift.setDirection(DcMotorSimple.Direction.FORWARD);

        Lift.setPower(0);
    }

    public void ManualLift() {
        if (elevator > 0){
            Lift.setDirection(DcMotorSimple.Direction.FORWARD);
            Lift.setPower(0.5);
            liftPosition = Lift.getCurrentPosition();
            telemetry.addData("Lift Position" , liftPosition);
            telemetry.update();
        }
        if (elevator < 0){
            Lift.setDirection(DcMotorSimple.Direction.REVERSE);
            Lift.setPower(0.5);
            liftPosition = Lift.getCurrentPosition();
            telemetry.addData("Lift Position" , liftPosition);
            telemetry.update();
        }

    }

}


