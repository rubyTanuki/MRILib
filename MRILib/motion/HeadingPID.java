package MRILib.motion;

import MRILib.util.*;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HeadingPID extends PID{
    
    public HeadingPID(double p, double i, double d){
        super(p,i,d);
    }
    
    @Override
    public double update(double current){
        double error = Mathf.angleWrap(targetVal - current);
        
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
    
    // public double update(double current1, double current2){
    //     return (Math.abs(current1-targetVal) < Math.abs(current2-targetVal)?update(current1):update(current2));
    // }
    

}

