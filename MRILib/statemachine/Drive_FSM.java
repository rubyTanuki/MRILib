package MRILib.statemachine;

import MRILib.managers.*;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.ArrayList;

public class Drive_FSM
{
    private Bot bot;
    private PIDController pid = null;
    ArrayList<BotState> steps = new ArrayList<>();

    public int currentStep = 0;

    public Drive_FSM(Bot bot, PIDController pid){
        this.bot = bot;
        this.pid = pid;
    }

    public void start(){
        steps.get(currentStep).start();
    }

    public void update(){
        steps.get(currentStep).update();
    }

    public void end(){
        steps.get(currentStep).end();
    }

    public void nextState(){
        currentStep++;
        start();
    }

    //MOVETO OVERLOADS
    
    public void moveTo(double x, double y, double theta, Runnable command){
        moveTo(x, y, theta, command, 5); }

    public void moveTo(double x, double y, double theta, double timeout){ 
        moveTo(x, y, theta, null, timeout); }

    public void moveTo(double x, double y, double theta){ 
        moveTo(x, y, theta, null, 5); }

    public void moveTo(Pose2D pose){ 
        moveTo(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.DEGREES)); }

    //MOVETO BASE METHOD
    
    public double lastX = 0;
    public double lastY = 0;
    public double lastAngle = 0;
    
    public void moveTo(double x, double y, double theta, Runnable command, double timeout){
        steps.add(new BotState("MOVE_TO:: (" + x + ", " + y + ") " + theta + "°", false){
            double tx = x;
            double ty = y;
            double tAngle = theta;
            ElapsedTime timer;
            
            @Override
            void start(){
                if(command!=null)command.run();
                pid.moveTo(tx,ty,tAngle);
                lastX = tx;
                lastY = ty;
                lastAngle = tAngle;
                timer = new ElapsedTime();
            }
            @Override
            void update(){
                pid.update();
                Pose2D curPos = pid.odo.getPosition();
                double deltaX = tx - curPos.getX(DistanceUnit.INCH);
                double deltaY = ty - curPos.getY(DistanceUnit.INCH);
                double deltaAngle = tAngle + curPos.getHeading(AngleUnit.DEGREES);
                deltaAngle = Mathf.angleWrap(deltaAngle);
                double dist = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
                
                if(dist < 2 && Math.abs(deltaAngle) < 3){
                    nextState();
                }
                if(timer.seconds() > timeout)
                    nextState();
                
            }
        });
    }

    //WAITFORSECONDS OVERLOADS

    public void waitForSeconds(double seconds){
        waitForSeconds(seconds, null); }
    public void run(Runnable command){
        waitForSeconds(0, command); }

    //WAITFORSECONDS BASE METHOD

    public void waitForSeconds(double seconds, Runnable command){
        steps.add(new BotState("WAIT_FOR_SECONDS:: (" + seconds + ")", false){
            ElapsedTime timer;

            @Override
            void start(){
                if(command!=null)command.run();
                pid.moveTo(lastX,lastY,lastAngle);
                timer = new ElapsedTime();
            }
            @Override
            void update(){
                pid.update();
                if(timer.seconds() > seconds) nextState();
            }
        });
    }
}