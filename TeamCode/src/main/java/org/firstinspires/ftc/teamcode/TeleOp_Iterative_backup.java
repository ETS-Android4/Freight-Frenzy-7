package org.firstinspires.ftc.teamcode;

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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

    /**
     * This file contains an example of an iterative (Non-Linear) "OpMode".
     * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
     * The names of OpModes appear on the menu of the FTC Driver Station.
     * When an selection is made from the menu, the corresponding OpMode
     * class is instantiated on the Robot Controller and executed.
     *
     * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
     * It includes all the skeletal structure that all iterative OpModes contain.
     *
     * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
     */
    //BackUp file
    @TeleOp(name="TeleOp BackUp Mode", group="Iterative Opmode")
//@Disabled
    public class TeleOp_Iterative_backup extends OpMode
    {
        // Declare OpMode members.
        DriveTrain MecDrive = new DriveTrain();
        private ElapsedTime runtime = new ElapsedTime();
        Intake intake = new Intake();
        CarouselDuck spinner = new CarouselDuck();
        Lift lift = new Lift();

        boolean aPrev = false;
        boolean dpdPrev = false;
        boolean dpuPrev = false;

        //private double Drive = 0;
        //private double Strafe = 0;
        //private double Turn = 0;

        /*
         * Code to run ONCE when the driver hits INIT
         */
        @Override
        public void init() {
            telemetry.addData("Status", "Initializing");

            MecDrive.init(hardwareMap);
            intake.init(hardwareMap);
            lift.init(hardwareMap);
            spinner.init(hardwareMap);




            // Tell the driver that initialization is complete.
            telemetry.addData("Status", "Initialized");
        }

        /*
         * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
         */
        @Override
        public void init_loop() {
        }

        /*
         * Code to run ONCE when the driver hits PLAY
         */
        @Override
        public void start() {
            runtime.reset();
        }

        /*
         * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
         */
        @Override
        public void loop() {
            // GamePad Inputs
            MecDrive.drive = -gamepad1.left_stick_y; //-1.0 to 1.0
            MecDrive.strafe = gamepad1.left_stick_x; //-1.0 to 1.0
            MecDrive.turn  =  gamepad1.right_stick_x; //-1.0 to 1.0

            intake.PickUp = gamepad2.right_bumper; //intake toggle
            intake.Drop = gamepad2.left_bumper; // out_take toggle
            intake.stopIntake = gamepad2.dpad_down; // remove
            intake.reverse = gamepad2.dpad_up; //toggle
            intake.freightCatch = gamepad2.x;

            spinner.armOut = gamepad1.a;
            spinner.armIn = gamepad1.b;
            spinner.duckSpinner = gamepad1.right_bumper; //toggle
            spinner.duckSpinnerRev = gamepad1.left_bumper; //toggle
            spinner.stopSpinner = gamepad1.y; //remove


       /* if(gamepad2.dpad_down && !dpdPrev) {
            lift.LiftStepDown();
        }
        dpdPrev = gamepad2.dpad_down;

        if(gamepad2.dpad_up && !dpuPrev) {
            lift.LiftStepUp();
        }
        dpuPrev = gamepad2.dpad_up;*/


            //if (gamepad2.a && !aPrev) {
            //    lift.elevatorLow = gamepad2.a;
            //}
            lift.elevatorLow = gamepad2.a;
            lift.elevatorMid = gamepad2.b;
            lift.elevatorHigh = gamepad2.y;

            //  Robot Functions

            MecDrive.MecanumDrive();
            intake.intake();
            spinner.carouselDuck();
            lift.ManualLift();



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)",MecDrive.leftFrontPower,MecDrive.rightFrontPower);
            telemetry.addData("Lift Position" , lift.liftPosition);
            telemetry.update();
            //
            /**liftPosition = Lift.getCurrentPosition();
             telemetry.addData("Lift Position" , liftPosition);
             telemetry.update();
             */
        }

        /*
         * Code to run ONCE after the driver hits STOP
         */
        @Override
        public void stop() {
        }

    }

