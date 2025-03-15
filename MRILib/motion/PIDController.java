package MRILib.motion;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;
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
    
    public double maxAngSpeed = .25; //.85
    public double maxSpeed = .3; //.9
    
    
    ElapsedTime timer = null;
    
    public boolean nextState = false;
    
    public PIDController(Bot bot, Telemetry telemetry){
        this.bot = bot;
        this.telemetry = telemetry;
    }
    public PIDController(ArmBotEasy bot, Telemetry telemetry){
        this.bot = (Bot)bot;
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

    public double update(){

        // //retrieving the current postiion from the pinpoint odometry
        // Pose2D curPos = bot.getPosition();
        // double curX = curPos.getX(DistanceUnit.INCH);
        // double curY = curPos.getY(DistanceUnit.INCH);
        // double botHeading = Math.toRadians(-curPos.getHeading(AngleUnit.DEGREES)); //radian
        
        // //getting the x and y pid results
        // double xVal = xPID.update(curX);
        // double yVal = yPID.update(curY);

        // //updating x and y variables for telemetry
        // targetY = yPID.targetVal;
        // targetX = xPID.targetVal;

        // // creating, clamping, and PIDing the theta power
        // double inputTheta = Math.atan2(yVal, xVal);
        // double inputPower = Range.clip(Math.sqrt(Math.abs(xVal * xVal + yVal * yVal)), -1, 1);
        // telemetry.addData("Input power", inputPower);
        // //double inputPower = Range.clip(Math.sqrt(Math.abs(xVal * xVal + yVal * yVal)), -maxSpeed, maxSpeed);
        // double inputTurn = -Mathf.angleWrap(targetRadians - botHeading);
        // if(Math.abs(inputTurn) > Math.toRadians(2)){
        //     inputTurn = thetaPID.update(inputTurn);
        //     inputTurn = Range.clip(inputTurn, -maxAngSpeed, maxAngSpeed);
        // }
        // else{
        //     inputTurn = 0;
        // }
        
        // // math for power calculations
        // double sin = Math.sin(inputTheta + Math.PI/4 - botHeading);
        // double cos = Math.cos(inputTheta + Math.PI/4 - botHeading);
        
        // // calculating power for drive motors
        // double maxed = Math.max(Math.abs(cos), Math.abs(sin));
        // double frontLeft = inputPower * (cos/maxed) - inputTurn;
        
        // double frontRight = inputPower * (sin/maxed) + inputTurn;
        
        // double backLeft = inputPower * (sin/maxed) - inputTurn;
        
        // double backRight = inputPower * (cos/maxed) + inputTurn;
        
        // telemetry.addData("frontLeft", frontLeft);
        // telemetry.addData("frontRight", frontRight);
        // telemetry.addData("backLeft", backLeft);
        // telemetry.addData("backRight", backRight);
        
        
        // //calculating max for clamping
        // double max1 = Math.max(Math.abs(frontLeft), Math.abs(frontRight));
        // double max2 = Math.max(Math.abs(backLeft), Math.abs(backRight));
        // double max3 = Math.max(max1, max2);
        
        
        // //applying power to drive motors
        
        // if(max3>1){
        //     bot.getFL().setPower((frontLeft / max3)*maxSpeed);
        //     bot.getFR().setPower((frontRight / max3)*maxSpeed);
        //     bot.getBL().setPower((backLeft / max3)*maxSpeed);
        //     bot.getBR().setPower((backRight / max3)*maxSpeed);
        // }else{
        //     bot.getFL().setPower(frontLeft*maxSpeed);
        //     bot.getFR().setPower(frontRight*maxSpeed);
        //     bot.getBL().setPower(backLeft*maxSpeed);
        //     bot.getBR().setPower(backRight*maxSpeed);
        // }
        
        // return max3;
        
        Pose2D curPos = bot.getPosition();
        double curX = curPos.getX(DistanceUnit.INCH);
        double curY = curPos.getY(DistanceUnit.INCH);
        double botHeading = Math.toRadians(curPos.getHeading(AngleUnit.DEGREES)); //radian
        
        double xVal = xPID.update(curX);
        double yVal = yPID.update(curY);

        targetY = yPID.targetVal;

        double inputTheta = Math.atan2(yVal, xVal);
        double inputPower = Range.clip(Math.sqrt(Math.abs(xVal * xVal + yVal * yVal)), -.9, .9);
        //double inputTurn = Mathf.angleWrap(targetAngle - botHeading);
        double inputTurn = -Mathf.angleWrap(targetRadians - botHeading);
        if(Math.abs(inputTurn) > Math.toRadians(2)){
            inputTurn = thetaPID.update(inputTurn);
            inputTurn = Range.clip(inputTurn, -maxAngSpeed, maxAngSpeed);
        }
        else{
            inputTurn = 0;
        }
        
        double sin = Math.sin(inputTheta + Math.PI/4 - botHeading);
        double cos = Math.cos(inputTheta + Math.PI/4 - botHeading);
        
        double maxed = Math.max(Math.abs(cos), Math.abs(sin));
        inputPower *= maxSpeed;
        double frontLeft = inputPower * (cos/maxed * (1/maxSpeed)) - inputTurn;
        double frontRight = inputPower * (sin/maxed * (1/maxSpeed)) + inputTurn;
        double backLeft = inputPower * (sin/maxed * (1/maxSpeed)) - inputTurn;
        double backRight = inputPower * (cos/maxed * (1/maxSpeed)) + inputTurn;
        
        double max1 = Math.max(Math.abs(frontLeft), Math.abs(frontRight));
        double max2 = Math.max(Math.abs(backLeft), Math.abs(backRight));
        double max = Math.max(1, Math.max(max1, max2));
        
        bot.getFL().setPower(frontLeft / max);
        bot.getFR().setPower(frontRight / max);
        bot.getBL().setPower(backLeft / max);
        bot.getBR().setPower(backRight / max);
        return max;
    }
    
    public void moveTo(double x, double y, double degrees)
    { // sets the target position to move towards
        xPID.setTarget(-x);
        yPID.setTarget(-y);
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
    public double getCurrentDegrees(){
        return Math.toDegrees(Math.toRadians(-bot.getPosition().getHeading(AngleUnit.DEGREES)));
    }
    public double getCurrentRadians(){
        return Math.toRadians(-bot.getPosition().getHeading(AngleUnit.DEGREES));
    }
    
    
}
