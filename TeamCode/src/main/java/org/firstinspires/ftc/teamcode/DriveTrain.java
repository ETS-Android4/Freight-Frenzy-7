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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public double drive = 0;
    public double strafe = 0;
    public double turn = 0;

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

        LFDrive.setDirection(DcMotor.Direction.FORWARD);
        RFDrive.setDirection(DcMotor.Direction.REVERSE);
        LBDrive.setDirection(DcMotor.Direction.FORWARD);
        RBDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        LFDrive.setPower(0);
        RFDrive.setPower(0);
        LBDrive.setPower(0);
        RBDrive.setPower(0);



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

// End of Class
 }

