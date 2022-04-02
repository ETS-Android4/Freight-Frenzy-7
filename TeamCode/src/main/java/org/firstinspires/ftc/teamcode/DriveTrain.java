/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 */
public class DriveTrain
{
    /* Public OpMode members. */
    public DcMotor LFDrive = null;
    public DcMotor RFDrive = null;
    public DcMotor LBDrive = null;
    public DcMotor RBDrive = null;
    public double leftFrontPower = 0;
    public double rightFrontPower = 0;
    public double leftBackPower = 0;
    public double rightBackPower = 0;
    public double LFSpeed;
    public double RFSpeed;
    public double LBSpeed;
    public double RBSpeed;
    public int LFPosition [];
    public int RFPosition [];
    public int LBPosition [];
    public int RBPosition [];
    public double drive = 0;
    public double strafe = 0;
    public double turn = 0;
    public Orientation lastAngles = new Orientation();
    public double currAngle = 0.0;

    private int LFMove;
    private int RFMove;
    private int LBMove;
    private int RBMove;
    private final double kDrive = 1;
    private final double kStrafe = 1;
    private final double kTurn = 1;



    BNO055IMU imu;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();


    /* Constructor */
    public DriveTrain(){

    }


    /* Initialize Hardware interfaces */


    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        LFDrive  = ahwMap.get(DcMotor.class, "LFDrive");
        RFDrive = ahwMap.get(DcMotor.class, "RFDrive");
        LBDrive  = ahwMap.get(DcMotor.class, "LBDrive");
        RBDrive = ahwMap.get(DcMotor.class, "RBDrive");

        LFDrive.setDirection(DcMotor.Direction.REVERSE);
        RFDrive.setDirection(DcMotor.Direction.FORWARD);
        LBDrive.setDirection(DcMotor.Direction.REVERSE);
        RBDrive.setDirection(DcMotor.Direction.FORWARD);

        LFPosition [4] = LFDrive.getCurrentPosition();
        RFPosition [4] = RFDrive.getCurrentPosition();
        LBPosition [4] = LBDrive.getCurrentPosition();
        RBPosition [4] = RBDrive.getCurrentPosition();

       //  This can be used to shift the array one to the left:  Arrays.stream(LFPosition).skip(1);

        // Set all motors to zero power
        LFDrive.setPower(0);
        RFDrive.setPower(0);
        LBDrive.setPower(0);
        RBDrive.setPower(0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


    }

    //Update motor position
    @RequiresApi(api = Build.VERSION_CODES.N) // required for array shift code
    public void updateRobotPosition(){
        Arrays.stream(LFPosition).skip(1);
        Arrays.stream(RFPosition).skip(1);
        Arrays.stream(LBPosition).skip(1);
        Arrays.stream(RBPosition).skip(1);

        LFPosition [4] = LFDrive.getCurrentPosition();
        RFPosition [4] = RFDrive.getCurrentPosition();
        LBPosition [4] = LBDrive.getCurrentPosition();
        RBPosition [4] = RBDrive.getCurrentPosition();

        LFMove = LFPosition [4] - LFPosition [3];
        RFMove = RFPosition [4] - RFPosition [3];
        LBMove = LBPosition [4] - LBPosition [3];
        RBMove = RBPosition [4] - RBPosition [3];

        double Drive = kDrive * (LFMove + RFMove + LBMove + RBMove);
        double Strafe = kStrafe * (LBMove + RFMove - LFMove - RBMove);
        double Turn = kTurn * (LFMove + LBMove - RFMove - RBMove);

        double RobotMove [] = new double[0];
        RobotMove [0] = Drive * Math.sin(currAngle)+Strafe * Math.cos(currAngle);
        RobotMove [1] = Drive * Math.cos(currAngle)+Strafe * Math.sin(currAngle);
        RobotMove [2] = Turn;
        double RobotPosition [] = {0, 0, 0}; // X, Y, Angle

        RobotPosition [0]= RobotPosition [0] + RobotMove[0];
        RobotPosition [1]= RobotPosition [1] + RobotMove[1];
        RobotPosition [2]= currAngle;

        /* Alternate code
        for  (int i = 0; i < 2; i++){
            RobotPosition[i] = RobotPosition[i] + RobotMove[i];
        }
        */

    }

    //Set power to all motors
    public void setAllPower(double p){setMotorPower(p,p,p,p);}

    public void setMotorPower(double lF,double rF,double lB,double rB){
        LFDrive.setPower(lF);
        RFDrive.setPower(rF);
        LBDrive.setPower(lB);
        RBDrive.setPower(rB);
    }
    public void MecanumDrive(){

        // Put Mecanum Drive math and motor commands here.

        double dPercent = Math.abs(drive) / (Math.abs(drive) + Math.abs(strafe) + Math.abs(turn));
        double sPercent = Math.abs(strafe) / (Math.abs(drive) + Math.abs(turn) + Math.abs(strafe));
        double tPercent = Math.abs(turn) / (Math.abs(drive) + Math.abs(turn) + Math.abs(strafe));

        rightFrontPower    = (drive * dPercent) + (-strafe * sPercent) + (-turn * tPercent);
        rightBackPower   = (drive * dPercent) + (strafe * sPercent) + (-turn * tPercent);
        leftFrontPower    = (drive * dPercent) + (strafe * sPercent) + (turn * tPercent);
        leftBackPower   = (drive * dPercent) + (-strafe * sPercent) + (turn * tPercent);

        // Send calculated power to wheels
        LFDrive.setPower(leftFrontPower);
        RFDrive.setPower(rightFrontPower);
        LBDrive.setPower(leftBackPower);
        RBDrive.setPower(rightBackPower);




    }
    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //imu is the internal gyro and things
        currAngle = 0;
    }
// End of Class
 }

