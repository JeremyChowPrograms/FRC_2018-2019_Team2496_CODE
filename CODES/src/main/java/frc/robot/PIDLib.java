package frc.robot;

public class PIDLib {
    long deltaT;
    long prevT;
    double kP;
    double kI;
    double kD;
    double integLimit;
    double integData;
    double prevError = 0, prevTime = 0;
    
    public PIDLib(double kP, double kI, double kD, double integLimit) {
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.integLimit = integLimit;
    }
    
    public void updateSpeed(double kP, double kI, double kD, double integLimit){
          this.kP = kP;
          this.kI = kI;
          this.kD = kD;
          this.integLimit = integLimit;
    }
    
   
  
  
    public double doPID(double error) {
      deltaT = (System.nanoTime() - prevT);
      prevT = System.nanoTime();
      
      double P = error * kP;
      double I = 0.0f; //you dont need this right now
      double D = (error-prevError)/deltaT * kD;
      prevError = error;
      return (P + I + D); 
    }
  
}