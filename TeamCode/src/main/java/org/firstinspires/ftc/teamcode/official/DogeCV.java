package org.firstinspires.ftc.teamcode.official;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
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

public class DogeCV {
    private OpenCvCamera phoneCam;
    private WABOTPipeline skyStoneDetector;
    private HardwareMap map;
    private Point translation = null;

    public DogeCV(HardwareMap map){
        this.map = map;
        init();
    }

    public void init() {
        int cameraMonitorViewId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        phoneCam.openCameraDevice();

        skyStoneDetector = new WABOTPipeline();
        phoneCam.setPipeline(skyStoneDetector);
    }

    public void startStreaming(){
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }
}