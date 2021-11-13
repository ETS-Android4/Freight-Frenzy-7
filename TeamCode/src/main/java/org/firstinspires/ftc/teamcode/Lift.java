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
    public boolean elevatorLow;
    public boolean elevatorMid;
    public boolean elevatorHigh;
    public final double MaxPower = 0.5;

    public final int low = -50;
    public final int mid = -450;
    public final int high = -1000;
    private final int mult = 538;



    /* Constructor */
    public Lift(){

    }
    /* Initialize Hardware interfaces */

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        Lift  = ahwMap.get(DcMotorEx.class, "Lift");

        Lift.setDirection(DcMotorSimple.Direction.FORWARD);

        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Lift.setPower(0);
    }

    public void ManualLift() {
            Lift.setPower(MaxPower);
            liftPosition = Lift.getCurrentPosition();
        //  18-36 lowest position
            if (elevatorLow) {
                Lift.setTargetPosition(low);
                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        if (elevatorMid){
            Lift.setTargetPosition(mid);
            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (elevatorHigh){
            Lift.setTargetPosition(high);
            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        //  -469 middle position?
        //  -1000 top position?


    }

}


