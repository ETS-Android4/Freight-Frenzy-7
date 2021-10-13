package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class TestOpenCV1 extends OpMode {
    OpenCvCamera Webcam = null;

    static double ringcount = 0;   // this is for the old season but I still put it in

    @Override
    public void init() {

    }
    @Override
    public void init_loop(){
        int cameraMonitorViewId = HardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "Id", HardwareMap.appContext.getPackageName());
        Webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        Webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {

            }

            @Override
            public void onError(int errorCode) {

            }
        })
            @Override
                    public void onopened(){
                Webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        }

        );


    }

    @Override
    public void loop() {

    }
}
