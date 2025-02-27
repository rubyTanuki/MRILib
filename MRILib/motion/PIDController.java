package MRILib.motion;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;
import Main.util.Mathf;
import org.firstinspires.ftc.teamcode.*;

/*
 * This file manages the three pid equations made at init for x, y, and turning movements respectively
 * It uses the calculated power output by each pid in order to determine power needed
 * on each motor to move in a strait line to the target position. Instructions on how to tune the PIDs
 * can be found in the example Autonomous file
 */


public class PIDController {
    private Bot bot = null;

    public PID xPID;
    public PID yPID;
    public PID thetaPID;

    public double targetX = 0;
    public double targetY = 0;
    public double targetRadians = 0; //radians
    
    public double maxAngSpeed = .85;
    public double maxSpeed = .9;
    
    
    ElapsedTime timer = null;
    
    public boolean nextState = false;
    
    public PIDController(Bot bot){
        this.bot = bot;
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

    public double update(){

        //retrieving the current postiion from the pinpoint odometry
        Pose2D curPos = bot.getPosition();
        double curX = curPos.getX(DistanceUnit.INCH);
        double curY = curPos.getY(DistanceUnit.INCH);
        double botHeading = Math.toRadians(curPos.getHeading(AngleUnit.DEGREES)); //radian
        
        //getting the x and y pid results
        double xVal = xPID.update(curX);
        double yVal = yPID.update(curY);

        //updating x and y variables for telemetry
        targetY = yPID.targetVal;
        targetX = xPID.targetVal;

        // creating, clamping, and PIDing the theta power
        double inputTheta = Math.atan2(yVal, xVal);
        double inputPower = Range.clip(Math.sqrt(Math.abs(xVal * xVal + yVal * yVal)), -maxSpeed, maxSpeed);
        double inputTurn = -Mathf.angleWrap(targetRadians - botHeading);
        if(Math.abs(inputTurn) > Math.toRadians(2)){
            inputTurn = thetaPID.update(inputTurn);
            inputTurn = Range.clip(inputTurn, -maxAngSpeed, maxAngSpeed);
        }
        else{
            inputTurn = 0;
        }
        
        // math for power calculations
        double sin = Math.sin(inputTheta + Math.PI/4 - botHeading);
        double cos = Math.cos(inputTheta + Math.PI/4 - botHeading);
        
        // calculating power for drive motors
        double maxed = Math.max(Math.abs(cos), Math.abs(sin));
        double frontLeft = inputPower * (cos/maxed * (1/maxSpeed)) - inputTurn;
        double frontRight = inputPower * (sin/maxed * (1/maxSpeed)) + inputTurn;
        double backLeft = inputPower * (sin/maxed * (1/maxSpeed)) - inputTurn;
        double backRight = inputPower * (cos/maxed * (1/maxSpeed)) + inputTurn;
        
        //calculating max for clamping
        double max1 = Math.max(Math.abs(frontLeft), Math.abs(frontRight));
        double max2 = Math.max(Math.abs(backLeft), Math.abs(backRight));
        double max = Math.max(1, Math.max(max1, max2));
        
        //applying power to drive motors
        bot.getFL().setPower(frontLeft / max);
        bot.getFR().setPower(frontRight / max);
        bot.getBL().setPower(backLeft / max);
        bot.getBR().setPower(backRight / max);
        return max;
    }
    
    public void moveTo(double x, double y, double degrees)
    { // sets the target position to move towards
        xPID.setTarget(x);
        yPID.setTarget(y);
        targetRadians = -Math.toRadians(degrees);
    }

    public void setMaxSpeed(double s){
        maxSpeed = s;
    }
    public void setMaxAngSpeed(double s){
        maxAngSpeed = s;
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
}
