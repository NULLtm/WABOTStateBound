package org.firstinspires.ftc.teamcode.official;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

public class WABOTController {
    private Gamepad g1;
    private Gamepad g2;

    private boolean[] checkBools = {true, true, true, true, true, true, true, true};
    private String[] buttonStrs = {"a", "b", "x", "y", "dpad_up", "dpad_down", "dpad_left", "dpad_right"};


    public WABOTController(Gamepad g1, Gamepad g2){
        this.g1 = g1;
        this.g2 = g2;
    }

    private int findIndex(String str){
        for(int i = 0; i < buttonStrs.length; i++){
            if(buttonStrs[i].equals(str)){
                return i;
            }
        }
        return 0;
    }

    public boolean returnToggleA(Gamepad g){
        int index = findIndex("a");
        if(g.a && checkBools[index]){
            checkBools[index] = false;
            return g.a;
        } else if(!checkBools[index] && !g.a){
            checkBools[index] = true;
        }
        return false;
    }
    public boolean returnToggleB(Gamepad g){
        int index = findIndex("b");
        if(g.b && checkBools[index]){
            checkBools[index] = false;
            return g.b;
        } else if(!checkBools[index] && !g.b){
            checkBools[index] = true;
        }
        return false;
    }
    public boolean returnToggleX(Gamepad g){
        int index = findIndex("x");
        if(g.x && checkBools[index]){
            checkBools[index] = false;
            return g.x;
        } else if(!checkBools[index] && !g.x){
            checkBools[index] = true;
        }
        return false;
    }
    public boolean returnToggleY(Gamepad g){
        int index = findIndex("y");
        if(g.y && checkBools[index]){
            checkBools[index] = false;
            return g.y;
        } else if(!checkBools[index] && !g.y){
            checkBools[index] = true;
        }
        return false;
    }
    public boolean returnToggleDPadDown(Gamepad g){
        int index = findIndex("dpad_down");
        if(g.dpad_down && checkBools[index]){
            checkBools[index] = false;
            return g.dpad_down;
        } else if(!checkBools[index] && !g.dpad_down){
            checkBools[index] = true;
        }
        return false;
    }
    public boolean returnToggleDPadUp(Gamepad g){
        int index = findIndex("dpad_up");
        if(g.dpad_up && checkBools[index]){
            checkBools[index] = false;
            return g.dpad_up;
        } else if(!checkBools[index] && !g.dpad_up){
            checkBools[index] = true;
        }
        return false;
    }
    public boolean returnToggleDPadLeft(Gamepad g){
        int index = findIndex("dpad_left");
        if(g.dpad_left && checkBools[index]){
            checkBools[index] = false;
            return g.dpad_left;
        } else if(!checkBools[index] && !g.dpad_left){
            checkBools[index] = true;
        }
        return false;
    }
    public boolean returnToggleDPadRight(Gamepad g){
        int index = findIndex("dpad_right");
        if(g.dpad_right && checkBools[index]){
            checkBools[index] = false;
            return g.dpad_right;
        } else if(!checkBools[index] && !g.dpad_right){
            checkBools[index] = true;
        }
        return false;
    }
}
