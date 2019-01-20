package frc.robot;

public class PIDLib {
    private double kp, ki, kd;
    private double target, error = 0.0d;
    private double errorprev;
    private double integral = 0.0d, integral_lim, derivative;

    public PIDLib(double kp, double ki, double kd, double integral_lim) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.integral_lim = integral_lim;
    }

    public double PID(double input, double target) {
        this.target = target;
        this.errorprev = this.error;
        this.error = this.target - input;
        if (this.error < integral_lim && this.error > -integral_lim) {
            integral += this.error;
        }
        this.derivative = this.error - this.errorprev;
        return this.kp * this.error + this.ki * this.integral + this.kd * this.derivative;
    }
}