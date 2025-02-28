
/*
 * This file contains the finite state machine for controlling your robot's arm with states and transitions
 * 
 * The init method contains the initialization of all states, which can then be accessed at runtime
 * by DriveFSM in order to control the arm, or controlled by teleop with individual setState() calls
 * 
 * The protected class ArmState is implemented at the bottom, extending BotState with a transition system
 * and a way to save initialized states for later use
 * 
 * In order for the state machine to react to dynamic information throughout the match (i.e. encoder changes and method calls),
 * you must call armSM.update() in your opMode in the loop, after calling bot.update() to update the opMode member manager class
 * 
 * For a robot with a transfer system, or multiple arms, I suggest creating multiple ArmFSM classes and seperating the logic between them,
 * as this allows for the easiest management of two seperate systems without making your state machines difficult to read and implement
 * You can also make a manager class for both state machines that will call setState on both classes at once for QOL and readability
 */


package MRILib.statemachine;

import org.firstinspires.ftc.teamcode.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.HashMap;


public class ArmFSM
{
    //opmode member manager
    private ArmBot bot; 

    //currently active state
    private BotState currentState; 

    //constructor
    public Arm_FSM(ArmBot bot){
        this.bot = bot;
        init();
        currentState = BotState.get("DEFAULT"); //starting on default state
    }


    public void start()
    { // runs once at the state's initialization
        currentState.start(); 
    }
    public void update()
    { // runs every repeat loop on the main thread
        currentState.update(); 
    }
    public void end()
    { // runs once at the end of the state or before transitioning to another state
        currentState.end(); 
    }


    public void setState(String name)
    { // setting currentState to another saved state
        setState(ArmState.get(name));
    }
    public void setState(ArmState state)
    { // setting currentState to another saved state
        if(state!=currentState){
            currentState.end();
            currentState = state;
            start();
        }
    }

    public void transition()
    { // checks all transition conditions and transitions if any are true
        HashMap<String, Boolean> transitions = currentState.getTransitions();
        for(String targetState : transitions.keySet()){
            if(transitions.get(targetState)){
                setState(targetState);
                break;
            }
        }
    }

    private void init(){
        // initializing and saving all arm states

        // ArmState constructor adds them to a static hashmap, so they can be created at init but 
        // referenced at runtime, also helps with readability

        new ArmState("DEFAULT"){ //replace "DEFAULT" with any string, becomes the name of the state
            // declare variables here
            // i.e. ElapsedTime timer;

            @Override
            void start(){
                // logic to be run once at first loop of the state
                // initialize variables and transitions here 
                // i.e. timer = new ElapsedTime();
            }
            @Override
            void update(){
                // init and update transition conditions here
                // i.e. setTransition(String targetState, boolean condition);

                transition(); // checking all transitions to see if the state machine should move to another

                // logic to be run every loop while state is active
            }
            @Override
            void end(){
                // logic to be run once after the last loop of the state
            }
        };
    }
    
}

class ArmState extends BotState{

    // static hashmap with all initialized states
    private static HashMap<String, BotState> states = new HashMap<>();
    public static BotState get(String s)
    { // returning a previously saved state
        try {
            return states.get(s);    
        } catch (Exception e) {
            return states.get("DEFAULT");
        }
    }
    public static void clearSavedStates()
    { // clearing the hashmap of all saved states for static garbage collecting
        states.clear();
    }

    public ArmState(){
        super();
        states.put("DEFAULT", this);
    }
    public ArmState(String name){
        super(name);
        states.put(name, this);
    }
    public ArmState(String name, boolean save){
        super(name);
        if(save) states.put(name, this);
    }

    public void setDefault(String state)
    { // setting the default state to any initialized state
        states.put("DEFAULT", states.get(state));
    }


    private HashMap<String, Boolean> transitions = new HashMap<>();
    public void setTransition(String targetState, boolean condition){
        transitions.put(targetState, condition);
    }
    public HashMap<String, Boolean> getTransitions(){
        return transitions;
    }
}

