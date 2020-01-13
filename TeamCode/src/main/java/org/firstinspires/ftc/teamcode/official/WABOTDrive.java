package org.firstinspires.ftc.teamcode.official;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

public class WABOTDrive extends Drive {
    @Override
    public Localizer getLocalizer() {
        return null;
    }

    @Override
    public void setLocalizer(Localizer localizer) {

    }

    @Override
    protected double getRawExternalHeading() {
        return 0;
    }

    @Override
    public void setDrivePower(Pose2d pose2d) {

    }

    @Override
    public void setDriveSignal(DriveSignal driveSignal) {

    }
}
