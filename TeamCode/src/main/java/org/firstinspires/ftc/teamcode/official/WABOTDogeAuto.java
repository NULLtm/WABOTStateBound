package org.firstinspires.ftc.teamcode.official;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Autonomous(name="DOGEAUTO")
public class WABOTDogeAuto extends LinearOpMode {

    DogeCV doge;

    @Override
    public void runOpMode() throws InterruptedException {

        doge = new DogeCV(hardwareMap);

        doge.startStreaming();

        waitForStart();

        run();
    }

    private void run(){
        while(opModeIsActive()){

        }
    }
}
