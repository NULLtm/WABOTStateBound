package org.firstinspires.ftc.teamcode.angleDrifter;

public class Vector2D {
    public double x, y;

    public Vector2D(double x, double y){
        this.x = x;
        this.y = y;
    }

    public static double distance(Vector2D a, Vector2D b){
        double distance = Math.sqrt(Math.pow(a.x-b.x, 2) + Math.pow(a.y-b.y, 2));
        return distance;
    }

    public static double normalizeValue(double value){
        double x = value;
        if(x > 0){
            x = 1;
        } else if(x < -1)
            x = -1;
        else if(Math.abs(x) < 0.001)
            x = 0;
        return x;
    }

    public static void moveToPosition(Vector2D a, Vector2D b, double percentMovement){
        double d = distance(a, b);

        if(d > 5){
            double dX = b.x - a.x;

            if(dX > 1)
                dX = 1;
            else if(dX < -1)
                dX = -1;
            else
                dX = 0;


            dX *= percentMovement;

            a.x += dX;

            double dY = b.y - a.y;

            if(dY > 0)
                dY = 1;
            else if(dY < 0)
                dY = -1;
            else
                dY = 0;

            dY *= percentMovement;

            a.y += dY;
        }
    }

    public Vector2D normalized(){
        Vector2D v = new Vector2D(x, y);
        if(x > 1){
            v.x = 1;
        }
        if(x < -1){
            v.x = -1;
        }
        if(y > 1){
            v.y = 1;
        }
        if(y < -1){
            v.y = -1;
        }

        return v;
    }
}
