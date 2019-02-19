package frc.robot;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

public class ChassisControl {
    public SpeedController fl, fr, bl, br;

    public ChassisControl(int fl, int fr, int bl, int br) {
        this.fl = new Talon(fl);
        this.fr = new Talon(fr);
        this.bl = new Talon(bl);
        this.br = new Talon(br);
    }

    public void tankDrive(double left, double right) {
        fl.set(left);
        bl.set(left);
        fr.set(right);
        br.set(-right);
    }

    public void arcadeDrive(double x, double y) {
        double left = y - x;
        double right = y + x;
        fl.set(left);
        bl.set(left);
        fr.set(right);
        br.set(-right);
    }
}