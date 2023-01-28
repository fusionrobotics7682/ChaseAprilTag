package frc.robot.triggers;

import edu.wpi.first.math.filter.Debouncer;

public class PidTrigger {
    
    Debouncer debouncer = new Debouncer(0.6);

    public boolean isFinished(boolean input){
        return debouncer.calculate(input);
    }
}
