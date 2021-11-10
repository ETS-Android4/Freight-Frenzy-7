package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCVWebcam2 {
    //public varibles
    public OpenCvCamera webcam;
    public int TeamEleLoc;
    public HardwareMap hwMap = null;
    public Mat outPut = new Mat();

    //private variables

    private final int RectLX = 20;
    private final int RectRX = 160;
    private final int RectLY = 150;
    private final int RectRY = 150;
    private final int RectLwidth = 60;
    private final int RectLheight = 60;
    private final int RectRwidth = 60;
    private final int RectRheight = 60;

    private final  Rect rectL = new Rect(RectLX, RectLY, RectLwidth, RectLheight);
    private final Rect rectR = new Rect(RectRX, RectRY, RectRwidth, RectRheight);
    private final Scalar rectanglecolor = new Scalar(0, 0, 0);

    private Mat LeftCrop = new Mat();
    private Mat RightCrop = new Mat();
    private Scalar LeftAverageR;
    private double FinaLeftAverageR;
    private double FinalRightAverageR;
    private Scalar RightAverageR;



    /* Constructor */
    public OpenCVWebcam2() {

    }
    /* Initialize Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new OpenCVWebcam2.SamplePipeline());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }



    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input)
        {
            input.copyTo(outPut);

            Imgproc.rectangle(outPut, rectL, rectanglecolor, 2);
            Imgproc.rectangle(outPut, rectR, rectanglecolor, 2);
            LeftCrop = input.submat(rectL);
            RightCrop = input.submat(rectR);

            // channel 1 = green

            Core.extractChannel(LeftCrop, LeftCrop, 1);
            Core.extractChannel(RightCrop, RightCrop, 1);

            LeftAverageR = Core.mean(LeftCrop);
            FinaLeftAverageR = LeftAverageR.val[0];

            RightAverageR = Core.mean(RightCrop);
            FinalRightAverageR = RightAverageR.val[0];

            if (FinalRightAverageR < 150 && FinaLeftAverageR < 150) {
                //team element = left
                TeamEleLoc = 0;
            }

            if (FinaLeftAverageR > 150){
                //team element = center
                TeamEleLoc = 1;
            }
            if (FinalRightAverageR > 150){
                //team element = right
                TeamEleLoc = 2;
            }


            return outPut;

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
