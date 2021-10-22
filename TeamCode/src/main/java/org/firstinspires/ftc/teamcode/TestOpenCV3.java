package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name="Drive To Target", group = "Concept")
@Disabled
public class TestOpenCV3 extends LinearOpMode
{
    OpenCvCamera Webcam;

    SamplePipeline pipeline;

    @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "Id", hardwareMap.appContext.getPackageName());
        Webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SamplePipeline();
        Webcam.setPipeline(pipeline);
        //Webcam.setPipeline(new ShippingElementDetectingPipeline());
        Webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                         @Override
                                         public void onOpened() {
                                             Webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                                         }

                                         @Override
                                         public void onError(int errorCode) {

                                         }
                                     });


      //telemetry.addData(">", "Press Play to start");
     // telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("type", pipeline.getType());
            telemetry.addData("Average", pipeline.getAverage());
            telemetry update();
            sleep (50);
        }
    }
}
