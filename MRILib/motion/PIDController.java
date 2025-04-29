package MRILib.motion;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;
import java.util.function.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import MRILib.managers.*;
import MRILib.util.Mathf;

/*
 * This file manages the three pid equations made at init for x, y, and turning movements respectively
 * It uses the calculated power output by each pid in order to determine power needed
 * on each motor to move in a strait line to the target position. Instructions on how to tune the PIDs
 * can be found in the example Autonomous file
 */

public class PIDController {
    private Bot bot = null;
    private Telemetry telemetry;

    public PID xPID;
    public PID yPID;
    public PID thetaPID;

    public double targetX = 0;
    public double targetY = 0;
    public double targetRadians = 0; //radians
    
    public Supplier<Double> maxAngSpeed = ()-> .6; //.85
    public Supplier<Double> maxSpeed; //.9
    public double POWER_SCALE = 1;
    
    
    ElapsedTime timer = null;
    
    public boolean nextState = false;
    
    public PIDController(Bot bot, Telemetry telemetry){
        maxSpeed = ()->.5;
        this.bot = bot;
        this.telemetry = telemetry;
    }

    public void setPID(PID x, PID y){
        xPID = x;
        yPID = y;
    }
    public void setTurnPID(PID theta){
        thetaPID = theta;
    }

    public void start(){
        xPID.start();
        yPID.start();
        thetaPID.setTarget(0);
    }

    public void update() {
        // Get the current pose and heading (field-centric)
        Pose2D curPos = bot.getPosition();
        double curX = curPos.getX(DistanceUnit.INCH);
        double curY = curPos.getY(DistanceUnit.INCH);
        double botHeading = Math.toRadians(bot.getHeading());  // current heading in radians
    
        // Get field-centric PID outputs for x and y (translation)
        double xVal = xPID.update(curX);  // how far we are from target X
        double yVal = yPID.update(curY);  // how far we are from target Y
        
        double thetaVal = -thetaPID.update(botHeading);
        thetaVal = Range.clip(thetaVal, -1.0, 1.0);
        
        double max = Math.max(xVal, yVal);
        
        // clamping translation while keeping ratio and
        // sending field centric power call to be translated to robot centric
        if(max>1)
            bot.driveFieldXYW(xVal/max*maxSpeed.get(), yVal/max*maxSpeed.get(), thetaVal*maxAngSpeed.get());
        else
            bot.driveFieldXYW(xVal*maxSpeed.get(), yVal*maxSpeed.get(), thetaVal*maxAngSpeed.get());
    }

    public void moveTo(double x, double y, double degrees)
    { // sets the target position to move towards
        xPID.setTarget(x);
        targetX = x;
        yPID.setTarget(y);
        targetY = y;
        targetRadians = Math.toRadians(degrees);
        thetaPID.setTarget(targetRadians);
    }

    public void setMaxSpeed(double s){
        maxSpeed = ()-> s;
    }
    public void setMaxAngSpeed(double s){
        maxAngSpeed = ()-> s;
    }


    // getters
    public double getTargetX(){
        return targetX;
    }
    public double getTargetY(){
        return targetY;
    }
    public double getTargetRadians(){
        return targetRadians;
    }
    public double getTargetDegrees(){
        return -Math.toDegrees(targetRadians);
    }
    public double getCurrentDegrees(){
        return Math.toDegrees(Math.toRadians(-bot.getPosition().getHeading(AngleUnit.DEGREES)));
    }
    public double getCurrentRadians(){
        return Math.toRadians(-bot.getPosition().getHeading(AngleUnit.DEGREES));
    }
    
    
}
