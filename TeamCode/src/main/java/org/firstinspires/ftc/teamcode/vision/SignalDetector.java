package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class SignalDetector {
    OpenCvCamera camera;
    SignalDetectorPipeline pipeline;

    public SignalDetector(HardwareMap hwMap) {
        int id = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        WebcamName name = hwMap.get(WebcamName.class, "camera");
        this.camera = OpenCvCameraFactory.getInstance().createWebcam(name, id);
        this.pipeline = new SignalDetectorPipeline();
    }
    public void init() {
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);}
            @Override
            public void onError(int errorCode) {}
        });
    }
    public void end() {
        camera.stopStreaming();
    }
    public int getCaseDetected() {
        return pipeline.getCaseDetected();
    }
}