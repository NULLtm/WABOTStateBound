package org.firstinspires.ftc.teamcode.official;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;

public class AutoPathFinder {

    private final double TRACK_WIDTH = 16;
    private Double[] robotPosition = {0d, 0d, 0d};

    private DcMotor[] driveTrainMotors = new DcMotor[4]; // FL, FR, BL, BR
    private DcMotor[] externalEncoders = new DcMotor[3]; // left, right, horizontal;

    private final double DISTANCE_TOLERANCE = 5;

    private double lastLEncoder;
    private double lastREncoder;
    private double lastSEncoder;

    private PIDFController turn, drive;
    private PIDCoefficients turnC, driveC;


    public AutoPathFinder(DcMotor[] motors, DcMotor[] encoders, double startHeading){
        robotPosition[2] = startHeading;
        if(motors.length == 4 && encoders.length == 3){
            driveTrainMotors[0] = motors[0];
            driveTrainMotors[1] = motors[1];
            driveTrainMotors[2] = motors[2];
            driveTrainMotors[3] = motors[3];
            externalEncoders[0] = encoders[0];
            externalEncoders[1] = encoders[1];
            externalEncoders[2] = encoders[2];
        }

        turn = new PIDFController(turnC);
        drive = new PIDFController(driveC);
    }

    private void goToPoint(double x, double y, double speed, double turnSpeed){
        lastLEncoder = externalEncoders[0].getCurrentPosition();
        lastREncoder = externalEncoders[1].getCurrentPosition();
        lastSEncoder = externalEncoders[2].getCurrentPosition();

        double OGangleOfPoint = Math.atan2(y - robotPosition[1], x - robotPosition[0]);

        turn.setInputBounds(-1 * Math.PI, Math.PI);
        turn.setOutputBounds(-turnSpeed, turnSpeed);
        turn.setTargetPosition(OGangleOfPoint);
        drive.setTargetPosition(0);
        drive.setOutputBounds(-speed, speed);

        while(Math.hypot(x - robotPosition[0], y - robotPosition[1]) > DISTANCE_TOLERANCE) {
            updatePosition();
            double angleOfPoint = Math.atan2(y - robotPosition[1], x - robotPosition[0]);
            double headingOfRobot = convertAngle(robotPosition[2]);
            double distance = Math.hypot(x - robotPosition[0], y - robotPosition[1]);
            double differenceInAngle = wrapAngle(headingOfRobot - angleOfPoint);

            double v1 = 0, v2 = 0, v3 = 0, v4 = 0;

            double correction = turn.update(headingOfRobot);
            double correctionDrive = drive.update(Math.hypot(x - robotPosition[0], y - robotPosition[1]));

            v1 = correction*-1; // TODO: Find out if this direction is correct
            v2 = correction;
            v3 = correction*-1;
            v4 = correction;

            v1 += correctionDrive;
            v2 += correctionDrive;
            v3 += correctionDrive;
            v4 += correctionDrive;

            driveTrainMotors[0].setPower(v1);
            driveTrainMotors[1].setPower(v2);
            driveTrainMotors[2].setPower(v3);
            driveTrainMotors[3].setPower(v4);


            lastLEncoder = externalEncoders[0].getCurrentPosition();
            lastREncoder = externalEncoders[1].getCurrentPosition();
            lastSEncoder = externalEncoders[2].getCurrentPosition();
        }
    }


    private void updatePosition(){
        Double[] x = getPosition(externalEncoders[1].getCurrentPosition() - lastREncoder, externalEncoders[0].getCurrentPosition() - lastLEncoder, externalEncoders[2].getCurrentPosition() - lastSEncoder);
        lastLEncoder = externalEncoders[0].getCurrentPosition();
        lastREncoder = externalEncoders[1].getCurrentPosition();
        lastSEncoder = externalEncoders[2].getCurrentPosition();
        robotPosition[0] += x[0];
        robotPosition[1] += x[1];
        robotPosition[2] += x[2];
    }

    private Double[] getPosition(double rightPosition, double leftPosition, double strafePosition){
        double theta = (rightPosition - leftPosition) / TRACK_WIDTH;
        theta += Math.PI / 2;
        theta = wrapAngle(theta);
        double theta2 = theta + Math.PI / 2;
        theta2 = wrapAngle(theta2);
        double strafeX = strafePosition * Math.cos(theta2);
        double strafeY = strafePosition * Math.sin(theta2);
        double distance = (rightPosition + leftPosition) / 2;
        double x = distance * Math.cos(theta);
        double y = distance * Math.sin(theta);

        x += strafeX;
        y += strafeY;

        Double[] array = {x, y, theta, distance};

        return array;
    }

    private static double wrapAngle(double theta){
        while(theta > Math.PI * 2 || theta < 0) {
            if(theta < 0){
                theta += Math.PI * 2;
            } else if(theta > Math.PI * 2){
                theta -= Math.PI * 2;
            }
        }

        return theta;
    }

    private double convertAngle(double heading){
        if(heading > 180){
            heading -= 360;
        }

        return heading;
    }
}
