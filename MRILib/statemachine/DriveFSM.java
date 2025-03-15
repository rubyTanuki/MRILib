
/*
 * This file contains the finite state machine for controlling your robot's motion throughout autonomous
 * 
 * The state machine operates by taking in a list of steps from the opmode at compile time and
 * iterating through the steps during the looping period. when either a waitForSeconds or MoveTo is called, 
 * the state gets added to a list of states. Each of these states have a transition condition in the update
 * method which allows them to decide when to move to the next step.
 * 
 * All coordinates input to MoveTo should be using the FTC standard x y grid, in which each unit corresponds to
 * one inch
 * This means that when creating a new autonomous, all you need to know is which tile the robot needs to go to, 
 * and where the robot starts. I reccomend starting with excessive waitForSeconds after any arm movements 
 * or large moveTo calls while testing until you are very comfortable with the timing of the movements, as well as 
 * setting your PIDController's max speed to pretty low at first
 * 
 * All states are overloaded with a Runnable parameter, which allows for multithreading of any turret/arm controls. This means
 * to put the method calls of controlling an arm in parallel to the DriveFSM operations, as well as lining up these commands with
 * the steps of your autonomous, eliminating the need for any timers on arm state control or position thresholds.
 * The runnables are primarily made easy by use of lambdas (runnable literals) in the opmode. Any method call or single line of code
 * can be entered here, so any type of arm control can feasibly be substituted for my ArmFSM easily.
 * 
 * Examples of all the usage for this can be found in ExampleAuton and ExampleTeleop
*/


package MRILib.statemachine;

import MRILib.managers.*;
import MRILib.motion.PIDController;
import MRILib.util.Mathf;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.ArrayList;

public class DriveFSM
{
    private Bot bot;
    private PIDController pid = null;
    ArrayList<BotState> steps = new ArrayList<>();

    public int currentStep = 0;

    public DriveFSM(Bot bot, PIDController pid){
        this.bot = bot;
        this.pid = pid;
    }

    public void start()
    { // runs once at the state's initialization
        steps.get(currentStep).start();
    }

    public void update()
    { // runs every repeat loop on the main thread
        steps.get(currentStep).update();
    }

    public void end()
    { // runs once at the end of the state or before transitioning to next state
        steps.get(currentStep).end();
    }

    public void nextState()
    { // transitioning to the next state once target condition is met
        if (currentStep > steps.size()) return;
        end();
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
    
    public void moveTo(double x, double y, double theta, Runnable command, double timeout)
    {// Adding a new state to the step list to move to a position
        steps.add(new BotState("MOVE_TO:: (" + x + ", " + y + ") " + theta + "°")
        { // Overriding the base methods of BotState with logic specific to this state
            double tx = x;
            double ty = y;
            double tAngle = theta;
            ElapsedTime timer;
            
            @Override
            void start(){
                // Running parallel arm control
                if(command!=null)command.run(); 

                // Setting PID target position
                pid.moveTo(tx,ty,tAngle);

                // Saving old values
                lastX = tx;
                lastY = ty;
                lastAngle = tAngle;

                // Starting timer for timeout
                timer = new ElapsedTime();
            }
            @Override
            void update(){
                // Updating pid to set motor values
                pid.update();

                // Calculating distance from target position using odometry
                Pose2D curPos = bot.getPosition();
                double deltaX = tx - curPos.getX(DistanceUnit.INCH);
                double deltaY = ty - curPos.getY(DistanceUnit.INCH);
                double deltaAngle = tAngle + curPos.getHeading(AngleUnit.DEGREES);
                deltaAngle = Mathf.angleWrap(deltaAngle);
                double dist = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
                

                // Declaring the acceptable error to allow moving onto the next step
                if(dist < 2 && Math.abs(deltaAngle) < 3){
                    nextState();
                }
                // Moving onto the next step if timer is above a timeout
                //      Mostly used to not get stuck indefinitely in the case of odometry not hitting its target
                //      or robot getting stuck on something
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

    public void waitForSeconds(double seconds, Runnable command)
    { // Adding a new state to the step list to wait for a specified time
        steps.add(new BotState("WAIT_FOR_SECONDS:: (" + seconds + ")")
        { // Overriding botstate with wait logic
            ElapsedTime timer;

            @Override
            void start(){
                // Running parallel arm command
                if(command!=null)command.run();

                // Setting pid target to the last target
                pid.moveTo(lastX,lastY,lastAngle);
                timer = new ElapsedTime();
            }
            @Override
            void update(){
                // Updating pid to move back to rest position if pushed out of it
                pid.update();

                // Moving to next state if wait value has elapsed
                if(timer.seconds() > seconds) nextState();
            }
        });
    }
}