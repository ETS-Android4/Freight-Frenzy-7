/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name="Webcam", group="OpMode")
public class OpenCVWebcam extends LinearOpMode
{
    //public varibles
    public OpenCvCamera webcam;
    public int TeamEleLoc;



    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new SamplePipeline());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry

            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();
*/
            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if(gamepad1.a)
            {
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */
                webcam.stopStreaming();
                //webcam.closeCameraDevice();
            }

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            sleep(100);
        }
    }

    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input)
        {

            Mat outPut = new Mat();
            int Rect1X = 20;
            int Rect2X = 160;
            int Rect1Y = 150;
            int Rect2Y = 150;
            int Rectwidth = 60;
            int Rectheight = 60;
            int Rect2width = 60;
            int Rect2height = 60;
            Mat CenterCrop = new Mat();
            Mat RightCrop = new Mat();
            //Left location = 0, center location = 1, right location = 2
            int TeamEleLoc = -1;

            input.copyTo(outPut);
            Rect rect = new Rect(Rect1X, Rect1Y, Rectwidth, Rectheight);
            Rect rect2 = new Rect(Rect2X, Rect2Y, Rect2width, Rect2height);

            Scalar rectanglecolor = new Scalar(0, 0, 0);

            Imgproc.rectangle(outPut, rect, rectanglecolor, 2);
            Imgproc.rectangle(outPut, rect2, rectanglecolor, 2);
           CenterCrop = input.submat(rect);
            RightCrop = input.submat(rect2);

        // We think channel 0 (coi) may be blue.
            //Core.extractChannel(CenterCrop, CenterCrop, 0);
            //Core.extractChannel(RightCrop, RightCrop, 0);

            Core.extractChannel(CenterCrop, CenterCrop, 1);
            Core.extractChannel(RightCrop, RightCrop, 1);


            //Core.extractChannel(CenterCrop, CenterCrop, 2);
           //Core.extractChannel(RightCrop, RightCrop, 2);

            Scalar CenterAverageR = Core.mean(CenterCrop);
            double FinalCenterAverageR = CenterAverageR.val[0];

            Scalar RightAverageR = Core.mean(RightCrop);
            double FinalRightAverageR = RightAverageR.val[0];

            if (FinalRightAverageR < 150 && FinalCenterAverageR < 150) {
                //team element = left
                TeamEleLoc = 0;
            }

            if (FinalCenterAverageR > 150){
                //team element = center
                TeamEleLoc = 1;
            }
            if (FinalRightAverageR > 150){
                //team element = right
                TeamEleLoc = 2;
            }
            telemetry.addData("center square green channel", FinalCenterAverageR);
            telemetry.addData("right square green channel", FinalRightAverageR);
            telemetry.addData("Team Element Location", TeamEleLoc);
            telemetry.update();
            sleep(500);
            return outPut;

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            //return input;


            // write crop code for third square
            // write two square method code for comparing two of the squares
            // white three square method for determinng team element locationat highest value of three squares
            }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}
