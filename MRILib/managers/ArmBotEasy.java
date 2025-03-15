package MRILib.managers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static MRILib.BotValues.*;

public class ArmBotEasy extends Bot{

    // Declaring OpMode members
    private DcMotorEx slide;
    private DcMotorEx pivot;

    private Servo claw;
    private Servo claw_pitch;
    private Servo claw_roll;
    

    int slidePos = 0;
    int pivotPos = 0;

    int slidePosPrev = 0;
    int pivotPosPrev = 0;

    public ArmBotEasy(HardwareMap hm)
    { //constructor method
        super(hm);
        initMotors(hm);
        initServos(hm);
        //resetEncoders();
    }

    private void initMotors(HardwareMap hm)
    { //initialising motors
        slide = hm.get(DcMotorEx.class, "slide");
        pivot = hm.get(DcMotorEx.class, "pivot");
        
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        slide.setDirection(SLIDEDIR);
        pivot.setDirection(PIVOTDIR);
        
        resetEncoders();
    }

    private void initServos(HardwareMap hm)
    { //initialising servos
        claw_pitch = hm.get(Servo.class, "clawPitch");
        claw_roll = hm.get(Servo.class, "claw_roll");
        claw = hm.get(Servo.class, "claw");
    }
    
    public void setArmMode(DcMotor.RunMode mode)
    {
        slide.setMode(mode);
        pivot.setMode(mode);
    }
    
    public void resetEncoders()
    { //Resetting encoders to 0, keeping current mode
        super.resetEncoders(); //Bot.resetEncoders()
        DcMotorEx.RunMode slideMode = slide.getMode();
        DcMotorEx.RunMode pivotMode = pivot.getMode();

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide.setMode(slideMode);
        pivot.setMode(pivotMode);
    }

    public void updateEncoders()
    { //updates current and last encoder positions
        super.updateEncoders(); //Bot.updateEncoders
        slidePosPrev = slidePos;
        pivotPosPrev = pivotPos;

        slidePos = slide.getCurrentPosition();
        pivotPos = pivot.getCurrentPosition();
    }
    
    //getter methods
    public DcMotorEx getSlideMotor()    { return slide;     }
    public DcMotorEx getPivotMotor()    { return pivot;     }
    public int getSlidePos()            { return slidePos;  }
    public int getPivotPos()            { return pivotPos;  }

    public Servo getClawPitch()         { return claw_pitch;    }
    public Servo getClawRoll()          { return claw_roll;     }
    public Servo getClaw()              { return claw;          }
    public double getClawPitchPos()     { return claw_pitch.getPosition();  }
    public double getClawRollPos()      { return claw_roll.getPosition();   }
    public double getClawPos()          { return claw.getPosition();        }

    public void setClawPitch(double pos){
        claw_pitch.setPosition(pos);
    }
    public void setClawRoll(double pos){
        claw_roll.setPosition(pos);
    }
    public void setClaw(double pos){
        claw.setPosition(pos);
    }

    public void setPivot(int target){
        pivot.setTargetPosition(target);
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivot.setVelocity(6000);
    }

    public void setSlides(int target){
        slide.setTargetPosition(target);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setVelocity(6000);
    }

    public void setArm(int pivotTarget, int slideTarget){
        setPivot(pivotTarget);
        setSlides(slideTarget);
    }
}
