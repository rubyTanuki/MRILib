package MRILib.statemachine;

import org.firstinspires.ftc.teamcode.*;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;

public class BotState{
    public String name;
    public BotState(String n){
        name = n;
    }
    public BotState(){
        name = "DEFAULT";
    }

    void start(){};
    void update(){};
    void end(){};

    public String toString(){
        return name;
    }
}