package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

public class TestOpenCV1 extends OpMode {
    OpenCvCamera Webcam = null;

    static int barcodelocation = 0;   // this is for the old season but I still put it in

    @Override
    public void init() {

    }
    @Override
    public void init_loop(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "Id", hardwareMap.appContext.getPackageName());
        Webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        Webcam.setPipeline(new ShippingElementDetectingPipeline());
        Webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

        {
            @Override
            public void onOpened() {
                Webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) {

            }
            });
            }


    @Override
    public void loop() {

    }
}

class ShippingElementDetectingPipeline extends OpenCvPipeline {
    Mat outPut = new Mat();
    int Rect1X = 0;
    int Rect2X = 0;
    int Rect1Y = 0;
    int Rect2Y = 0;
    int Rectwidth = 119;
    int Rectheight = 220;
    int Rect2width = 119;
    int Rect2height = 220;
    Mat CenterCrop = new Mat();
    Mat RightCrop = new Mat();

    @Override

    public Mat processFrame(Mat input) {

        input.copyTo(outPut);
        Rect rect = new Rect(Rect1X, Rect1Y, Rectwidth, Rectheight);
        Rect rect2 = new Rect(Rect2X, Rect2Y, Rect2width, Rect2height);

        Scalar rectanglecolor = new Scalar(0, 0, 0);

        Imgproc.rectangle(outPut, rect, rectanglecolor, 2);
        CenterCrop = input.submat(rect);
        RightCrop = input.submat(rect2);

        Core.extractChannel(CenterCrop, CenterCrop, 2);
        Core.extractChannel(RightCrop, RightCrop, 2);

        Scalar CenterAverageR = Core.mean(CenterCrop);
        double FinalCenterAverageR = CenterAverageR.val[0];

        Scalar RightAverageR = Core.mean(RightCrop);
        double FinalRightAverageR = RightAverageR.val[0];

        if (FinalRightAverageR < FinalCenterAverageR) {

        }

        return outPut;
    }

}


