package org.firstinspires.ftc.teamcode.official;

//import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vector3;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.Locale;

public class OpenCVLoader {
    private OpenCvCamera phoneCam;
    private WABOTPipeline skyStoneDetector;
    private HardwareMap map;
    private Point translation = null;
    private final boolean RUN_ON_APP;

    public OpenCVLoader(HardwareMap map, boolean RUN_ON_APP){
        this.map = map;
        this.RUN_ON_APP = RUN_ON_APP;
        init();
    }

    public void init() {
        int cameraMonitorViewId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());

        if(RUN_ON_APP){
            phoneCam = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        } else {
            phoneCam = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName.class, "Webcam 1"));
        }

        phoneCam.openCameraDevice();

        skyStoneDetector = new WABOTPipeline();
        phoneCam.setPipeline(skyStoneDetector);
    }

    public void startStreaming(){
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }
}