package MRILib.managers;

import MRILib.managers.*;
import static MRILib.BotValues.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PivotArmController implements Runnable{
    private ArmBotFF bot;


    private final double k1 = 10.0; //LQR gain 1
    private final double k2 = 3.19374388; //LQR gain 2
    private final double kG = 1.0; //gravity gain
    private final double kD = 1.0; //friction gain

    private volatile double targetTheta; //radians
    private volatile double targetAngularVelocity;

    private volatile double currentArmTheta;
    private volatile double currentArmAngularVelocity;
    private double previousArmTheta;
    private double previousArmAngularVelocity;

    private volatile boolean running = true;

    public PivotArmController(ArmBotFF bot){
        //constructor
        this.bot = bot;
    }


    @Override 
    public void run(){
        long loopTime = 10;
        long nextLoopTime = System.nanoTime() + loopTime * 1_000_000;
        ElapsedTime timer = new ElapsedTime();
        double previousTime = timer.seconds();
        
        
        while(running){
            double currentTime = timer.seconds();
            double deltaTime = currentTime-previousTime;
            previousTime = currentTime;



            double thetaDot = (currentArmTheta - previousArmTheta) / deltaTime; //synching with discrete time
            double thetaDotDot = currentArmAngularVelocity / deltaTime;

            double thetaError = targetTheta - thetaDot;
            double angularVelocityError = targetAngularVelocity - thetaDotDot;

            double gravityFeedForward = kG * Math.cos(thetaDot);
            double frictionFeedforward = kD * thetaDotDot;
            double u_ff = gravityFeedForward + frictionFeedforward; //calculate u feedforward
            
            double u_fb = -(k1 * thetaError + k2 * angularVelocityError); //calculate u feedback (LQR)

            double u = u_ff + u_fb; //add feedback and feedforward voltages

            setMotorVoltage(Math.max(-12, Math.min(12, u)));




            while(System.nanoTime() < nextLoopTime){
                //wait until next loop time
            }
            nextLoopTime += loopTime * 1_000_000;
        }
        
    }

    public void setMotorVoltage(double voltage){
        //use voltage sensor to clamp volts to 0-12
        double clampMulti = 12.0/bot.getVoltage();
        //calculate 0-1 relative to current voltage output and the target voltage
        double powerValue = (1.0/voltage)*clampMulti;

        //apply power value to motor
        bot.setPivotPower(powerValue);
    }

    public synchronized void updateCurrentState(double armPos, double velocity){
        previousArmTheta = currentArmTheta;
        previousArmAngularVelocity = currentArmAngularVelocity;
        currentArmTheta = Math.toRadians(armPos/TICKS_PER_DEGREE_ARM);
        currentArmAngularVelocity = Math.toRadians(velocity/TICKS_PER_DEGREE_ARM); 
    }

    public void setTargetTheta(double theta, AngleUnit unit){
        targetTheta = unit==AngleUnit.DEGREES?Math.toRadians(theta):theta;
    }

    public void stop(){
        running = false;
    }
}
