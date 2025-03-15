package MRILib.managers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import MRILib.util.GoBildaPinpointDriver;
import static MRILib.BotValues.*;


/*
 * This file contains the initialization of the drive train motors, imu, 
 * odometry computer, and voltage sensor
 * as well as all getter, setter, and helper methods for any related values
 * 
 * It is a superclass for ArmBot, which implements the same helper methods but for
 * any arm and game unique motors, servos, and sensors
 * 
 * While the ArmBot class and its subclasses need to be different for every individual robot,
 * this class should not change besides a few initialization parameters
 * 
 * This file is specifically illustrates the manager and initialization class for a
 * 4-motor Omni-Directional (or Holonomic) Drive-train using GoBilda Mechanum wheels and the
 * GoBilda Pinpoint Odometry Computer I2C device. It can be easily modified, however, to instead utilize a 
 * custom odometry class using wheel encoders or a 3 dead-wheel odometry system by switching out the odo
 * variable and making sure all the method names line up between the calls on this file and the 
 * names on your odometry file.
 */

public class Bot{

    //creating motor variables to be initialized in initMotors()
    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;
    
    //creating odometry computer to be initialized in initOdo()
    public GoBildaPinpointDriver odo;

    //creating on-board IMU to be initialized in initIMU()
    public IMU imu;

    //creating voltage sensor object
    private VoltageSensor voltageSensor;

    //current and last position, calculated by the odometry computer
    private Pose2D currentPosition;
    private Pose2D lastPosition;
    
    //current motor positions
    int frontLeftPos  = 0;
    int frontRightPos = 0;
    int backLeftPos   = 0;
    int backRightPos  = 0;

    //previous motor positions
    int frontLeftPosPrev  = 0;
    int frontRightPosPrev = 0;
    int backLeftPosPrev   = 0;
    int backRightPosPrev  = 0;

    public Bot(HardwareMap hm)
    { //constructor
        initMotors(hm);
        initIMU(hm);
        initOdo(hm);
        voltageSensor = hm.voltageSensor.iterator().next();
    }

    private void initMotors(HardwareMap hm)
    { //initializing drive motors
        frontLeft = hm.get(DcMotorEx.class, "frontLeft");
        frontRight = hm.get(DcMotorEx.class, "frontRight");
        backLeft = hm.get(DcMotorEx.class, "backLeft");
        backRight = hm.get(DcMotorEx.class, "backRight");

        //setting the direction of the motors so all go forward when set to positive power
        setDirections();
    }

    public void initIMU(HardwareMap hardwareMap)
    { //initializing on board IMU

        //initializing the imu object using the hardwareMap from your OpMode
        imu = hardwareMap.get(IMU.class, "imu");

        // setting up the orientation of the control hub on the robot in order to
        // properly calculate the yaw, pitch, and roll of the robot using the gyroscope
        // first parameter takes a LogoFacingDirection enum value and the second parameter
        // takes a UsbFacingDirection value

        //example: 
        //  new RevHubOrientationOnRobot(LogoFacingDirection.LEFT, UsbFacingDirection.UP)
        IMU.Parameters params = new IMU.Parameters(
            new RevHubOrientationOnRobot(LOGO_DIR, USB_DIR));

        //initializing the imu with the specified orientation parameters
        imu.initialize(params);
    }

    public void initOdo(HardwareMap hardwareMap)
    { //initialize odometry

        //initializing the odometry object using the hardwareMap from your OpMode
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        // setting the init parameters of the two odometry wheels 
        // for refrence of how to determine these parameters, look to "ComputeOdo.java" in .util
        odo.setOffsets(96.0, 75.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // resetting the position and heading to zero (this can be reset to a different starting position
        // using setPosition())
        odo.resetPosAndIMU();
    }

    public void update(){
        //update robot position and encoder ticks in each opMode loop tick
        odo.update();
        lastPosition = currentPosition;
        currentPosition = getOdoPosition();
        updateEncoders();
    }

    public void updateEncoders()
    { //updating current position numbers

        //saving the current motor positions as the position in the last tick
        frontLeftPosPrev = frontLeftPos;
        frontRightPosPrev = frontRightPos;
        backLeftPosPrev = backLeftPos;
        backRightPosPrev = backRightPos;

        //updating the current motor positions to their new values
        frontLeftPos = frontLeft.getCurrentPosition();
        frontRightPos = frontRight.getCurrentPosition();
        backLeftPos = backLeft.getCurrentPosition();
        backRightPos = backRight.getCurrentPosition();
    }

    

    //getter methods
    public DcMotorEx getFL(){ return frontLeft;     }
    public DcMotorEx getFR(){ return frontRight;    }
    public DcMotorEx getBL(){ return backLeft;      }
    public DcMotorEx getBR(){ return backRight;     }
    public int getFLPos(){ return frontLeftPos;  }
    public int getFRPos(){ return frontRightPos; }
    public int getBLPos(){ return backLeftPos;   }
    public int getBRPos(){ return backRightPos;  }
    public int getFLPosPrev() { return frontLeftPosPrev;  }
    public int getFRPosPrev() { return frontRightPosPrev; }
    public int getBLPosPrev() { return backLeftPosPrev;   }
    public int getBRPosPrev() { return backRightPosPrev;  }

    public double getIMUHeading(){ return -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);}
    public void resetIMUHeading(){ imu.resetYaw(); }
    
    public Pose2D getPosition(){
        // returning the current odometry position output by the odometry computer
        return currentPosition;
    }
    public Pose2D getLastPosition(){
        // returning the odometry position as it was at the last update tick
        return lastPosition;
    }

    private Pose2D getOdoPosition(){
        // returns the odo position direct from the odometry computer
        // (this is private so that getPosition is used, which is properly synced with the update ticks)
        return odo.getPosition();
    }
    public double getX(){
        // returns the X value of the current odometry position
        return getPosition().getX(DistanceUnit.INCH);
    }
    public double getY(){
        // returns the Y value of the current odometry position
        return getPosition().getY(DistanceUnit.INCH);
    }
    public double getHeading(){
        // returns the heading value of the current odometry position using 
        // the odometry computer's IMU by default (it seems to be more consistent than the on-board IMU)
        return getPosition().getHeading(AngleUnit.DEGREES);
    }

    public synchronized double getVoltage()
    { // returning the current output voltage from the battery to the control hub
        return voltageSensor.getVoltage();
    }
    

    public void setPosition(double x, double y, double heading){
        //overloading setPosition to take an x, y, and heading instead of a pose for QOL
        setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading));
    }
    public void setPosition(Pose2D pos)
    { // overriding the current position read by the Gobilda Pinpoint Odometry Computer
        odo.setPosition(pos);
    }

    public void setDirections()
    { //setting the directions of the motors to their values specified in the static BotValues
        frontLeft.setDirection(LEFTDIR);
        frontRight.setDirection(RIGHTDIR);
        backLeft.setDirection(LEFTDIR);
        backRight.setDirection(RIGHTDIR);
    }
    
    public void setMode(DcMotor.RunMode mode)
    { //setting motors to an input runmode
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    public void resetEncoders()
    { //Resetting encoders to 0, keeping current mode

        //setting the previous encoder tick values to zero to reflect the new encoder reset
        resetPrevEncoders();

        //saving the runmode of the drive motors so that it can be returned to after the motors are reset
        DcMotor.RunMode mode = frontLeft.getMode();

        //stop and resetting the encoder values of each drive motor
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //reapplying the saved runmode
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    public void resetPrevEncoders(){
        frontLeftPosPrev = 0;
        frontRightPosPrev = 0;
        backLeftPosPrev = 0;
        backRightPosPrev = 0;
    }

    public void enableBrakeMode(boolean enabled)
    { // turning on or off brakemode
        DcMotor.ZeroPowerBehavior state = enabled ? BRAKE : FLOAT;
        frontLeft.setZeroPowerBehavior(state);
        frontRight.setZeroPowerBehavior(state);
        backLeft.setZeroPowerBehavior(state);
        backRight.setZeroPowerBehavior(state);
    }

    public void driveXYW(double rx, double ry, double rw)
    { // sets proportional power to drive motors

        //    The input x, y, and w represent a 2 dimensional vector for the intended
        //    direction of motion, as well as an intended power for rotation around 
        //    the z axis (spinning in a circle)
        //    
        //    this is typically input using the gamepad with -left_stick_x being jy, -left_stick_y being jx, and
        //    right_stick_x being jw
        //    
        //    calculates the power that needs to go to each of the four mechanum wheels
        //    in order to push the robot in the direction specified by the 2D vector, while applying further power
        //    to rotate the target amount
        //    
        //    these power values are then proportionally clamped to stay within a maximum power value of 1, as well
        //    as regulating the max speed using a voltage sensor multiplier
        

        // denom is the multiplier applied to the power calculations in order to 
        // ensure the motors all remain <= 1 but are still proportionally the same
        double denom = Math.max(Math.abs(rx)+Math.abs(ry)+Math.abs(rw),1);

        // dividing the power by a multiplier to counteract variance in motor voltage above 12 volts
        // ensuring that power doesnt get unexpectedly high, causing precise movements to get inaccurate 
        double voltageMulti = getVoltage() / 12;

        // adding and subtracting the x, y, and theta power values for each wheel to
        // push the robot in the vector direction made when combining all three powers
        double lfPower = ((rx - ry - rw) / denom) / voltageMulti;
        double rfPower = ((rx + ry + rw) / denom) / voltageMulti;
        double lbPower = ((rx + ry - rw) / denom) / voltageMulti;
        double rbPower = ((rx - ry + rw) / denom) / voltageMulti;
        
        // applying calculated vector powers to each motor
        frontLeft.setPower(lfPower);
        frontRight.setPower(rfPower);
        backLeft.setPower(lbPower);
        backRight.setPower(rbPower);
    }
    
    public void driveFieldXYW(double fx, double fy, double fw, double rot)
    { // rotate field orientation inputs to robot orientation

        //   takes the same inputs as driveXYW but rotates the power values for
        //   field centric driver control
        //
        //   field centric driving is a driver control mode which results in the inputs of the driver, i.e. x, y, and w
        //   pushing the robot relative to the field and driver instead of itself
        //   effectively causing movement to be the same regardless of the robot's current orientation in respect to yaw
        //
        //   although initially confusing and disorientating, this driver control mode offers beneficial maneuverability
        //   and at the end of the day makes more sense for the maneuvering that FTC robots must perform in their matches


        // getting the current robot's yaw, and converting it to a usable form to calculate the offset
        double theta = Math.toRadians(-getHeading());

        //calculating the rotated x power
        double rx = fx * Math.cos(-theta) - fy * Math.sin(-theta);
        //calculating the rotated y power
        double ry = fx * Math.sin(-theta) + fy * Math.cos(-theta);

        // inputting the new rotated x and y vector to driveXYW
        // passing through the rotation value (fw) as rotation around the robot's z axis
        // is unaffected by rotation of the directional power vector
        driveXYW(rx, ry, fw);
    }
}
