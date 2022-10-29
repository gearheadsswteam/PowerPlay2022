package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.VisionValueStorage.signalLower;
import static org.firstinspires.ftc.teamcode.vision.VisionValueStorage.signalMinArea;
import static org.firstinspires.ftc.teamcode.vision.VisionValueStorage.signalUpper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class SignalDetectorPipeline extends OpenCvPipeline implements SignalDetector {
    Mat process = new Mat();
    Mat output = new Mat();
    int caseDetected = 0;
    private OpenCvCamera cam;

    private LinearOpMode curOpMode = null;   //current opmode

    /* local OpMode members. */
    private HardwareMap hardwareMap = null;

    public SignalDetectorPipeline (LinearOpMode currrentOpMode, HardwareMap hwMap){
        this.curOpMode = currrentOpMode;
        this.hardwareMap = hwMap;
    }

    @Override
    public void initialize() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName name = hardwareMap.get(WebcamName.class, "camera");
        this.cam = OpenCvCameraFactory.getInstance().createWebcam(name, cameraMonitorViewId);
        cam.setPipeline(this);
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);;}
            @Override
            public void onError(int errorCode) {};
        });
        //cam.openCameraDevice();

        //cam.setFlashlightEnabled(true);
    }

    @Override
    public int detectSignal() {
        return caseDetected;
    }

    int count = 0;

    public Mat processFrame(Mat input) {
        curOpMode.telemetry.addData("process frame ", count++);
        curOpMode.telemetry.update();
        output = input.clone();
        int[] maxColor = {0, -1};
        for (int i = 0; i < signalLower.length; i++) {
            ArrayList<MatOfPoint> contours = new ArrayList<>();
            Rect maxRect = new Rect();
            Imgproc.cvtColor(input, process, Imgproc.COLOR_RGB2YCrCb);
            Core.inRange(process, signalLower[i], signalUpper[i], process);
            Imgproc.morphologyEx(process, process, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(process, process, Imgproc.MORPH_CLOSE, new Mat());
            Imgproc.GaussianBlur(process, process, new Size(15, 5), 0);
            Imgproc.findContours(process, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(output, contours, -1, new Scalar(0, 0, 0));
            for (MatOfPoint contour : contours) {
                Point[] contourArr = contour.toArray();
                Rect boundingRect = Imgproc.boundingRect(new MatOfPoint2f(contourArr));
                if (contourArr.length >= 15 && boundingRect.area() > signalMinArea && boundingRect.area() > maxRect.area()) {
                    maxRect = boundingRect.clone();
                }
            }
            if (maxRect.area() > maxColor[0]) {
                maxColor = new int[]{(int) maxRect.area(), i};
            }
            Imgproc.rectangle(output, maxRect, new Scalar(255, 255, 255));
            Imgproc.putText(output, "Color " + (i + 1) + " Area: " + maxRect.area(), new Point(10, 20 + 15 * i), 0, 0.5, new Scalar(255, 255, 255), 1);
        }
        caseDetected = maxColor[1] + 1;
        if (caseDetected == 0) {
            Imgproc.putText(output, "Signal not detected", new Point(10, 350), 0, 0.5, new Scalar(255, 255, 255), 1);
        } else {
            Imgproc.putText(output, "Case: " + caseDetected, new Point(10, 350), 0, 0.5, new Scalar(255, 255, 255), 1);
        }
        output.release();
        process.release();
        return input;
    }
}