package MRILib.motion;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {

    final double kP; // software defined spring pulling toward target
    final double kI; // force away from repeated motion (anti wobble)
    final double kD; // software defined dampener pushing away from target
    public double iLimit = 1;

    public double targetVal = 0;
    
    double errorSum = 0;
    public double errorSumTotal = .1;
    double lastTime = 0;
    double lastError = 0;
    
    ElapsedTime timer = null;
    
    public PID(double p, double i, double d){
        kP = p;
        kI = i;
        kD = d;
        timer = new ElapsedTime();
    }
    
    public void start(){
        errorSum = 0;
        lastError = 0;
        timer.reset();
    }
    
    public double update(double current){
        double error = targetVal - current;
        
        double deltaTime = timer.seconds() - lastTime;

        if (Math.abs(error) < iLimit) {
            errorSum += error * deltaTime;
        }
        if(errorSum < -errorSumTotal)
            errorSum = -errorSumTotal;
        if(errorSum > errorSumTotal)
            errorSum = errorSumTotal;
        

        double errorRate = (error - lastError) / deltaTime;

        double value = kP * error + kI * errorSum + kD * errorRate;
        
        
        
        lastTime = timer.seconds();
        lastError = error;
        
        return value;
    }
    
    public void setTarget(double target){
        targetVal = target;
    }
    
    public void setILimit(double limit){
        iLimit = limit;
    }
    

}

