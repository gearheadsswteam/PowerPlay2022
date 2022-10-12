package org.firstinspires.ftc.teamcode;
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
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
public class ShippingElementDetector {
    int redMultiplier;
    OpenCvCamera camera;
    OpenCvPipeline pipeline;
    String caseDetected = "C";
    public ShippingElementDetector(HardwareMap hwMap, int side) {
        int id = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        WebcamName name = hwMap.get(WebcamName.class, "camera");
        this.redMultiplier = side;
        this.camera = OpenCvCameraFactory.getInstance().createWebcam(name, id);
        this.pipeline = new OpenCvPipeline() {
            @Override
            public Mat processFrame(Mat input) {
                Mat process = new Mat();
                Mat output = input.clone();
                ArrayList<MatOfPoint> contours = new ArrayList<>();
                Scalar lower = new Scalar(0, 0, 0);
                Scalar upper = new Scalar(255, 120, 120);
                Rect maxRect = new Rect();
                int minAreaThreshold = 5000;
                Imgproc.cvtColor(input, process, Imgproc.COLOR_RGB2YCrCb);
                Core.inRange(process, lower, upper, process);
                Imgproc.morphologyEx(process, process, Imgproc.MORPH_OPEN, new Mat());
                Imgproc.morphologyEx(process, process, Imgproc.MORPH_CLOSE, new Mat());
                Imgproc.GaussianBlur(process, process, new Size(15, 5), 0);
                Imgproc.findContours(process, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
                Imgproc.drawContours(output, contours, -1, new Scalar(0, 0, 255));
                for (MatOfPoint contour : contours) {
                    Point[] contourArr = contour.toArray();
                    Rect boundingRect = Imgproc.boundingRect(new MatOfPoint2f(contourArr));
                    if (contourArr.length >= 15 && boundingRect.area() > minAreaThreshold && boundingRect.area() > maxRect.area()) {
                        maxRect = boundingRect.clone();
                    }
                }
                if (maxRect.x + maxRect.width / 2 > 240) {
                    if (redMultiplier == 1) {
                        caseDetected = "B";
                    } else {
                        caseDetected = "C";
                    }
                } else if (maxRect.x + maxRect.width / 2 > 0) {
                    if (redMultiplier == 1) {
                        caseDetected = "A";
                    } else {
                        caseDetected = "B";
                    }
                } else {
                    if (redMultiplier == 1) {
                        caseDetected = "C";
                    } else {
                        caseDetected = "A";
                    }
                }
                Imgproc.rectangle(output, maxRect, new Scalar(255, 0, 0));
                Imgproc.putText(output, "Area: " + maxRect.area(), new Point(10, 590), 0, 1, new Scalar(0, 0, 0), 2);
                Imgproc.putText(output, "Case: " + caseDetected, new Point(10, 630), 0, 1, new Scalar(0, 0, 0), 2);
                process.release();
                output.release();
                return input;
            }
        };
    }
    public void initialize() {
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);}
            @Override
            public void onError(int errorCode) {};
        });
    }
    public void end() {
        camera.stopStreaming();
    }
    public String caseDetected() {
        return caseDetected;
    }
}
