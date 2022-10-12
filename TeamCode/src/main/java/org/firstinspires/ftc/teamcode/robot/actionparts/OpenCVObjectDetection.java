package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.helpers.ContourPipeline;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;


public class OpenCVObjectDetection {

    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    double CrLowerUpdate = 150;
    double CbLowerUpdate = 120;
    double CrUpperUpdate = 255;
    double CbUpperUpdate = 255;

    double lowerruntime = 0;
    double upperruntime = 0;

    // Yellow Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 130, 0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 190.0, 100.0);


    private ContourPipeline myPipeline;
    private OpMode opMode;


    public void initialize(OpMode opMode) {
        this.opMode = opMode;
        hardwareMap = opMode.hardwareMap;
        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline


        webcam.setPipeline(myPipeline = new ContourPipeline());
        // Configuration of Pipeline
        myPipeline.ConfigurePipeline(30, 30, 30, 30, CAMERA_WIDTH, CAMERA_HEIGHT);
        myPipeline.ConfigureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        myPipeline.ConfigureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);
        // Webcam Streaming

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                opMode.telemetry.addData("Error in OPenCV with error code ", errorCode);
                opMode.telemetry.update();

            }
        });

        opMode.telemetry.update();
    }

    public String getObjectDetectionState() {

        if (myPipeline.error) {
            opMode.telemetry.addData("Exception: ", myPipeline.debug);
        }
        // Only use this line of the code when you want to find the lower and upper values, using Ftc Dashboard (https://acmerobotics.github.io/ftc-dashboard/gettingstarted)
        // testing(myPipeline);

        // Watch our YouTube Tutorial for the better explanation

        opMode.telemetry.addData("RectArea: ", myPipeline.getRectArea());
        opMode.telemetry.update();

        if (myPipeline.getRectArea() > 2000) {
            if (myPipeline.getRectMidpointX() > 400) {
                return CapstoneDetector.POSITION_C;
            } else if (myPipeline.getRectMidpointX() > 200) {
                return CapstoneDetector.POSITION_B;
            } else {
                return CapstoneDetector.POSITION_A;
            }
        }
        return "X";
    }
}


