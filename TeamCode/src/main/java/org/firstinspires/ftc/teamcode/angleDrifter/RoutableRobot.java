package org.firstinspires.ftc.teamcode.angleDrifter;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Vector3;

import org.firstinspires.ftc.teamcode.angleDrifter.RoutePoint;
import org.firstinspires.ftc.teamcode.official.WABOTImu;

import java.io.File;
import java.io.IOException;
import java.net.URL;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

public class RoutableRobot {

    public List<Vector2D> points = new ArrayList<>();


    private Vector2D robotPosition, originalRobotPosition;

    private final double RUN_SPEED = 1;
    private final double TURN_RADIUS = 0.5; /// Max: 1 Min: 0
    private final double ANGLE_TOLERANCE = 0.004;
    private final double DISTANCE_TOLERANCE = 30;
    private final double STRAFE_TOLERANCE = 0.5; // Max: 1 Min: 0


    private boolean getAngle = true;

    private double oGAngle = 0;

    private final double MAP_RATIO = 900 / 146.4; // UNITS: Pixels Per Inch
    private final Vector2D ROBOT_SIZE = new Vector2D(18, 18); // In INCHES

    private int quadrant = 4;
    private boolean hasI = false;

    private double pointOverallDistance, pointOverallAngle, originalRot;
    private boolean runOnce = true;

    private int currentPointI = 0;

    public void initPointsFromFile(){

        File f = AppUtil.getInstance().getSettingsFile("points.txt");
        PointRoster r = new PointRoster(f);

        String currentText = "", currentX = "", currentY = "";
        double x, y;
        boolean running = true;
        int currentI = 0;
        while(running){
            System.out.println("Running line now");
            try{
                currentText = r.readNextLine();

                if(currentText != null){
                    for(int i = 0; i < currentText.length(); i++){
                        if(currentText.charAt(i) == ','){
                            currentX = currentText.substring(0, i);
                            currentY = currentText.substring(i+2);
                        }
                    }
                    x = Double.parseDouble(currentX);
                    y = Double.parseDouble(currentY);

                    if(currentI==0){
                        robotPosition = new Vector2D(x, y);
                        originalRobotPosition = new Vector2D(x, y);
                    } else {
                        points.add(new Vector2D(x, y));
                    }
                } else {
                    running = false;
                }
            } catch(IOException e){
                running = false;
            }
            currentI++;
        }
    }

    public void runToPoint(WABOTImu imu, int index, DcMotor FLMotor, DcMotor FRMotor, DcMotor BLMotor, DcMotor BRMotor, DcMotor strafeOdometer, DcMotor leftOdometer, DcMotor rightOdometer) {
        for(Vector2D x: points){
            currentPointI = index;
            while(Vector2D.distance(points.get(currentPointI), robotPosition) > DISTANCE_TOLERANCE){
                double newX = Math.cos(getNeatRotation(imu.getHeading()))*leftOdometer.getCurrentPosition();
                double newY = Math.sin(getNeatRotation(imu.getHeading()))*leftOdometer.getCurrentPosition();
                robotPosition = new Vector2D(originalRobotPosition.x+newX, originalRobotPosition.y+newY);
                Vector2D v = points.get(currentPointI);

                if(getAngle){
                    getAngle = false;
                    oGAngle = Math.atan2(v.y-robotPosition.y, v.x-robotPosition.x) * -1;
                }

                double difference = Math.toRadians(imu.getHeading()) - oGAngle;

                if(!hasI && difference >= -1.35 && difference <= 1.35){
                    quadrant = 2;
                    hasI = true;
                } else if(!hasI && difference <= 4.5 && difference >= 1.8){
                    quadrant = 3;
                    hasI = true;
                } else if(!hasI && difference < 1.8 && difference > 1.35){
                    hasI = true;
                    quadrant = 0;
                } else if(!hasI && (difference < -1.35 && difference > -1*(Math.PI/2))||(difference < (3*Math.PI / 2) && difference > 4.5)){
                    hasI = true;
                    quadrant = 1;
                }

                if(runOnce){
                    runOnce = false;
                    pointOverallDistance = Vector2D.distance(points.get(currentPointI), robotPosition);
                    pointOverallAngle = difference;
                    originalRot = Math.toRadians(imu.getHeading());
                }

                if(quadrant == 2){
                    double change = Vector2D.normalizeValue(difference);

                    double changeFactor = RUN_SPEED;
                    double changeMove = changeFactor;

                    double currentDistance = Vector2D.distance(points.get(currentPointI), robotPosition);

                    if(Math.abs(difference) > ANGLE_TOLERANCE){
                        double magnitudeS = changeFactor;
                        double angleS = Math.toDegrees(oGAngle);
                        double v1 = 0, v2 = 0, v3 = 0, v4 = 0;
                        int quadrantS = 0;

                        if(angleS >= 0 && angleS <= 90){
                            quadrantS = 1;
                        } else if(angleS > 90 && angleS <= 180){
                            quadrantS = 2;
                        } else if(angleS > 180 && angleS <= 270){
                            quadrantS = 3;
                        } else if(angleS > 270 && angleS <= 360) {
                            quadrantS = 4;
                        }

                        if(magnitudeS != 0) {
                            if (quadrantS == 1) {
                                v1 = magnitudeS * ((angleS - 45) / 45);
                                v3 = magnitudeS;
                                v2 = magnitudeS;
                                v4 = magnitudeS * ((angleS - 45) / 45);
                            } else if (quadrantS == 2) {
                                v1 = magnitudeS;
                                v3 = magnitudeS * ((135 - angleS) / 45);
                                v2 = magnitudeS * ((135 - angleS) / 45);
                                v4 = magnitudeS;
                            } else if (quadrantS == 3) {
                                v1 = magnitudeS * ((225 - angleS) / 45);
                                v3 = -1 * magnitudeS;
                                v2 = -1 * magnitudeS;
                                v4 = magnitudeS * ((255 - angleS) / 45);
                            } else if (quadrantS == 4) {
                                v1 = -1 * magnitudeS;
                                v3 = -1 * magnitudeS * ((angleS - 315) / 45);
                                v2 = -1 * magnitudeS * ((angleS - 315) / 45);
                                v4 = -1 * magnitudeS;
                            } else if (quadrantS == 0) {
                                v1 = 0;
                                v2 = 0;
                                v3 = 0;
                                v4 = 0;
                            }

                            FLMotor.setPower(v2);
                            FRMotor.setPower(v1);
                            BLMotor.setPower(v3);
                            BRMotor.setPower(v4);
                        }
                    } else {
                        FLMotor.setPower(changeMove);
                        FRMotor.setPower(changeMove);
                        BLMotor.setPower(changeMove);
                        BRMotor.setPower(changeMove);
                    }
                }

                /*if(quadrant == 3){
                    System.out.println("Running backwards");
                    double change = Vector2D.normalizeValue(difference-Math.PI);
                    double bChange = Math.abs(difference)-Math.PI;

                    //System.out.println("Should turn in this direction: "+change);
                    //System.out.println("Angle difference: "+(Math.abs(difference)-Math.PI));
                    //System.out.println("Point angle: "+oGAngle);
                    //System.out.println("Rotation of robo: "+obj1.getDifferenceRotation());

                    double changeFactor = RUN_SPEED;
                    double changeMove = changeFactor;

                    if(Math.abs(bChange) > ANGLE_TOLERANCE){
                        obj1.rotZ += change*changeFactor*0.06;
                        changeMove = changeFactor+(changeFactor*TURN_RADIUS);
                    }

                    obj1.position.x -= forward.x*changeMove;
                    obj1.position.y -= forward.y*changeMove;
                }

                if(quadrant == 0){
                    System.out.println("Running strafe RIGHT");
                    double change, bChange;
                    change = difference-(Math.PI/2);

                    bChange = Math.abs(difference)-(Math.PI/2);

                    double changeFactor = RUN_SPEED;
                    double changeMove = changeFactor;

                    if(Math.abs(bChange) > ANGLE_TOLERANCE){
                        obj1.rotZ += change*changeFactor*0.06;
                        changeMove = changeFactor+(changeFactor*TURN_RADIUS);
                    }

                    obj1.position.x += right.x*changeMove;
                    obj1.position.y += right.y*changeMove;
                }
                if(quadrant == 1){
                    System.out.println("Running STRAFE LEFT");
                    double change = 0;
                    double bChange = 0;

                    if(difference < 0){
                        change = Vector2D.normalizeValue(difference+(Math.PI/2));
                        bChange = Math.abs(difference)+(Math.PI/2);
                    } else if(difference > 0){
                        change = Vector2D.normalizeValue(difference-(3*Math.PI/2));
                        bChange = Math.abs(difference)-(3*Math.PI/2);
                    }

                    double changeFactor = RUN_SPEED;
                    double changeMove = changeFactor;

                    if(Math.abs(bChange) > ANGLE_TOLERANCE){
                        obj1.rotZ += change*changeFactor*0.06;
                        changeMove = changeFactor+(changeFactor*TURN_RADIUS);
                    }

                    obj1.position.x -= right.x*changeMove;
                    obj1.position.y -= right.y*changeMove;
                }*/
            } /*else {
                hasI = false;
                quadrant = 4;
                getAngle = true;
                currentPointI++;
                runOnce = true;
            }*/
        }
    }

    private double getNeatRotation(double heading){
        if(heading < 0){
            heading += 360;
        }

        return heading;
    }

}
