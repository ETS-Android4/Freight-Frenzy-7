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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="Blue Duck Unit", group="Pushbot")
//@Disabled
public class Blue_Duck_Deliver_Park_In_Storage_Unit extends LinearOpMode {


    DriveTrain MecDrive = new DriveTrain();
    private ElapsedTime runtime = new ElapsedTime();
    Intake intake = new Intake();
    CarouselDuck spinner = new CarouselDuck();
    Lift lift = new Lift();
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    OpenCVWebcam2 Vision = new OpenCVWebcam2();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");

        MecDrive.init(hardwareMap);
        intake.init(hardwareMap);
        lift.init(hardwareMap);
        spinner.init(hardwareMap);

        Vision.init(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        telemetry.addData(">", "Robot Ready.");
        telemetry.update();

        while (!isStarted()) {
            //telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.addData("left square green channel", Vision.LeftMax);
            telemetry.addData("middle square green channel", Vision.MiddleMax);
            telemetry.addData("right square green channel", Vision.RightMax);
            //telemetry.addData("Adjusted Threshold", Vision.AdjustedThreshold);
            //telemetry.addData("Unadjusted Threshold", Vision.UnadjustedThreshold);
            telemetry.addData("Team Element Location", Vision.TeamEleLoc);
            telemetry.update();
            Vision.FinalTeamEleLoc = Vision.TeamEleLoc;
        }
        Vision.webcam.stopStreaming();
        telemetry.addData("Final Team Element Location", Vision.FinalTeamEleLoc);
        telemetry.update();

        //Duck Arm Stuff

        spinner.DuckArm.setPosition(spinner.arm);
        spinner.DuckSpinner.setPower(.7);
        spinner.carouselDuck();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.5)) {

            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();

        }
        //spinner.stopSpinner = true;
        spinner.DuckSpinner.setPower(0);
        spinner.DuckArm.setPosition(spinner.rest);

        sleep(2000);

//square to wall
        turn(-40);


//drive forward
        MecDrive.drive = 0.6;
        MecDrive.strafe = 0.0;
        MecDrive.turn = 0.0;
        MecDrive.MecanumDrive();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();

        }

//turn left
        turn(90);

//go forward
        MecDrive.drive = 0.6;
        MecDrive.strafe = 0.0;
        MecDrive.turn = 0.0;
        MecDrive.MecanumDrive();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .25)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }


//lift by vision
        lift.ManualLift();
        if (Vision.FinalTeamEleLoc == 0) {
            lift.Lift.setTargetPosition(lift.low);
            lift.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (Vision.FinalTeamEleLoc == 1) {
            lift.Lift.setTargetPosition(lift.mid);
            lift.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (Vision.FinalTeamEleLoc == 2) {
            lift.Lift.setTargetPosition(lift.high);
            lift.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

//go forward
        MecDrive.drive = 0.2;
        MecDrive.strafe = 0.0;
        MecDrive.turn = 0.0;
        MecDrive.MecanumDrive();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .15)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }


//drop freight
        intake.intake();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.1)) {
            //intake.Drop = true;
            intake.intake.setPower(.65);
        }
        //intake.stopIntake = true;
        intake.intake.setPower(0);

//going backwards
        MecDrive.drive = -0.4;
        MecDrive.strafe = 0.0;
        MecDrive.turn = 0.0;
        MecDrive.MecanumDrive();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

//Drop Lift
        lift.elevatorLow = true;

//Backup to wall
        MecDrive.drive = -0.6;
        MecDrive.strafe = 0.0;
        MecDrive.turn = 0.0;
        MecDrive.MecanumDrive();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.75)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

//strafe to unit
        MecDrive.drive = 0.0;
        MecDrive.strafe = -0.65;
        MecDrive.turn = 0.0;
        MecDrive.MecanumDrive();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

    }


//  Start Gyro methods copy
    double gyroTurnMin = 0.25;
    double gyroTurnMax = 1;
    public double getAngle(){
        Orientation orientation = MecDrive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;
        if(deltaAngle > 180){
            deltaAngle -= 360;
        } else if(deltaAngle <= -180){
            deltaAngle += 360;
        }
        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("Gyro", orientation.firstAngle);
        return currAngle;
    }

    public void turn(double degrees){
       MecDrive.resetAngle();
        double error = degrees;
        while (opModeIsActive() && Math.abs(error) > 2){
            double setPower = Range.clip(Math.abs(error)/180+gyroTurnMin, -gyroTurnMax, gyroTurnMax);
            double motorPower = (error < 0 ? -setPower: setPower);
            MecDrive.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }
        MecDrive.setAllPower(0);
        MecDrive.MecanumDrive();
    }

    // end gyro method copy
}