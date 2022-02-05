package org.firstinspires.ftc.teamcode.Mechanical;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift2 {

    public DcMotorEx LinearActuator;
    private int liftPosition;
    private int liftSetpoint;  // Would use if separate logic and motor commands
    private double liftPower;  // Would use if separate logic and motor commands
    //private double stickInput;

    HardwareMap hwMap = null;

    /*
    public double elevator = 0;
    public boolean elevatorLow = true;
    public boolean elevatorMid;
    public boolean elevatorHigh;
    public int ifRun = 0;
    */

    private final double MaxPower = 0.5;
    private final int maxIncrement = 20;
    private final int liftOffPoint = 50;

    private final int extHeight1 = 20;
    private final int extHeight2 = 350;//changed from -450 by MandJ
    private final int extHeight3 = 850;//changed from -1000 by MandM

 /*
    private final int mult = 538;
    public final int increment = 30;
    private boolean runLift = false;
*/


    /* Constructor */
    public Lift2(){

    }

    /* Initialize Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        LinearActuator = ahwMap.get(DcMotorEx.class, "LinearActuator");
        LinearActuator.setDirection(DcMotorSimple.Direction.FORWARD);
        LinearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinearActuator.setPower(0);
    }

    //  Run the lift based on gamepad or autonomous inputs
    public int ManualLift(double stickInput, boolean ext1, boolean ext2, boolean ext3) {
        liftPosition = - LinearActuator.getCurrentPosition();

        // This block determines the lift setpoint and motor power
        if (ext1 && liftPosition <= liftOffPoint){
            LinearActuator.setPower(0);
        }
        else if (ext1){
            LinearActuator.setTargetPosition(extHeight1);
            LinearActuator.setPower(MaxPower);
        }
        else if (ext2){
            LinearActuator.setTargetPosition(extHeight2);
            LinearActuator.setPower(MaxPower);
        }
        else if (ext3){
            LinearActuator.setTargetPosition(extHeight3);
            LinearActuator.setPower(MaxPower);
        }
        else if (stickInput > 0.1 || stickInput < 0.1){
            LinearActuator.setTargetPosition((int) (liftPosition + stickInput * maxIncrement));
            LinearActuator.setPower(MaxPower*stickInput);
        }

        return liftPosition;

        /*
        // This is an example using an input array instead of individual variables

        // The below arrays would be defined where the variables they are replacing are defined
        boolean ext[]; // use instead of ext1, ext2, ext3...
        int extHeight[];  // use instead of extHeight1, extHeight2, extHeight3...

        if (ext[1] && liftPosition <= liftOffPoint){
            LinearActuator.setPower(0);
        }
        else if (Arrays.asList(ext).contains(true)){
            for (int i = 0; i < Array.getLength(ext); i++){  // would need to confirm this covers the full array
                if(ext[i]){
                    LinearActuator.setTargetPosition(extHeight[1]);
                }
            }
            LinearActuator.setPower(MaxPower);
        }
        else if (stickInput > 0.1 || stickInput < 0.1){
            LinearActuator.setTargetPosition((int) (liftPosition + stickInput * maxIncrement));
            LinearActuator.setPower(MaxPower*stickInput);
        }

         */

    }

}


