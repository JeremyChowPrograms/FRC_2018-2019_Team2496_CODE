package frc.robot;

public class PIDLib{
    private double kp,ki,kd;
        private double target, error;
        private double errorprev;
        private double integral, integral_lim, derivative;
        public PIDLib(double kp, double ki, double kd, double integral_lim, double target){
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
            this.target = target;
        }
        public double PID(double input){
            this.errorprev = this.error;
            this.error = this.target - input;
            if(this.error < integral_lim && this.error > -integral_lim){
                integral+=this.error;
            }
            this.derivative = this.error - this.errorprev;
            return this.kp * this.error + this.ki * this.integral + this.kd * this.derivative;
        }
}