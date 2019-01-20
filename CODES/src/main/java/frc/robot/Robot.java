/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final int kJoystickPort = 0;
  private final int kJoystickPort2 = 1;
  private Joystick leftJoyStick, rightJoyStick;
  private ChassisControl chassis;
  private UsbCamera camera;
  private Encoder liftEncoder, leftE, rightE;
  private SpeedController liftMotor;
  private SpeedController claw;
  private boolean isTeleop = false;

  private ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  private Compressor compressor;
  //Light sensors for auto-correction
  private DigitalLib left, mid, right;

  private DoubleSolenoid lift1, lift2, intake;

  //False if not attached, will error if set true but cam not attached
  private boolean useCamera = false;
  private double liftHeight = 0.0d;

  public Robot() {
  }

  @Override
  public void robotInit() {
    //Ports according to design table
    chassis = new ChassisControl(3, 4, 1, 2);
    gyro.reset();
    gyro.calibrate();
    gyro.reset();
    //The params for constructors might change due to design and such, change if required
    liftEncoder = new Encoder(0, 1);
    leftE = new Encoder(2, 3);
    rightE = new Encoder(4, 5);
    liftEncoder.setDistancePerPulse(Math.PI / 200);
    leftE.setDistancePerPulse(6 * Math.PI / 200);
    rightE.setDistancePerPulse(6 * Math.PI / 200);
    leftJoyStick = new Joystick(kJoystickPort);
    rightJoyStick = new Joystick(kJoystickPort2);
    claw = new Talon(5);
    liftMotor = new Talon(6);
    compressor = new Compressor();
    compressor.setClosedLoopControl(true);
    lift1 = new DoubleSolenoid(0, 1);
    lift2 = new DoubleSolenoid(2, 3);
    intake = new DoubleSolenoid(4, 5);
    left = new DigitalLib(0, 1);
    mid = new DigitalLib(2, 3);
    right = new DigitalLib(4, 5);
    if (useCamera) {
      camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(800, 600);
      camera.setFPS(60);
    }
  }

  // Auton code here, used Init because only run once
  @Override
  public void autonomousInit() {
    encoderForward(72, 0.4);
  }

  //These are the macros that can be used in either auton or teleop
  private void leftTurn(double degrees, double speed) {
    gyro.reset();
    double doPid = 0;
    PIDLib pid = new PIDLib(0.008f, 0, 0.00001, 0);
    while (degrees + gyro.getAngle() > 5) {
      doPid = pid.PID(gyro.getAngle(), -degrees - 5);
      chassis.tankDrive(-speed - doPid, speed + doPid);
    }
    chassis.tankDrive(speed, -speed);
    Timer.delay(0.1);
    chassis.tankDrive(0, 0);
  }

  private void rightTurn(double degrees, double speed) {

    gyro.reset();
    double doPid = 0;
    PIDLib pid = new PIDLib(0.008f, 0, 0.00001, 0);
    while (degrees - 5 > gyro.getAngle()) {
      doPid = pid.PID(gyro.getAngle(), degrees + 5);
      chassis.tankDrive(speed + doPid, -speed - doPid);
    }
    chassis.tankDrive(-speed, speed);
    Timer.delay(0.1);
    chassis.tankDrive(0, 0);
  }

  private void encoderForward(double distance, double baseSpeed) {

    leftE.reset();
    rightE.reset();
    double kin = 0;
    double kinscale = 0.01;
    while ((leftE.getDistance() < distance || rightE.getDistance() < distance)) {
      chassis.tankDrive(baseSpeed - kin * kinscale, baseSpeed + kin * kinscale);
      kin = (leftE.getDistance() - rightE.getDistance());
    }
    chassis.tankDrive(-1, -1);
    Timer.delay(0.1);
    chassis.tankDrive(0, 0);
  }

  private void encoderBackward(double distance, double baseSpeed) {

    leftE.reset();
    rightE.reset();
    double kin = 0;
    double kinscale = 0.01;
    while ((leftE.getDistance() > distance || rightE.getDistance() > distance)) {
      chassis.tankDrive(-baseSpeed - kin * kinscale, -baseSpeed + kin * kinscale);
      kin = -(leftE.getDistance() - rightE.getDistance());
    }
    chassis.tankDrive(1, 1);
    Timer.delay(0.1);
    chassis.tankDrive(0, 0);
  }
  
  // Init Code
  @Override
  public void teleopInit() {
    isTeleop = this.isOperatorControl();
    {
      Thread t = new Thread() {
        @Override
        public void run() {
          //Doing PID for lift
          PIDLib pid = new PIDLib(0.3, 0, 0.03, 0.0);
          liftEncoder.reset();
          while (isTeleop) {
            liftMotor.set(pid.PID(liftEncoder.getDistance(), liftHeight));
          }
        }
      };
      t.start();
    }
  }

  @Override
  public void disabledInit() {
    isTeleop = false;
  }

  // Teleop code here
  @Override
  public void teleopPeriodic() {
    // Info, nop region
    {
      SmartDashboard.setDefaultNumber("Value", leftJoyStick.getZ());
      isTeleop = this.isOperatorControl();
    }
    // Op region
    claw.set(leftJoyStick.getZ());
    //Lift and intake pneumatics
    lift1.set(leftJoyStick.getRawButton(2) ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    lift2.set(leftJoyStick.getRawButton(3) ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    intake.set(leftJoyStick.getRawButton(4) ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    // Macro, Driving
    {
      //If hold button 3, will do auto-correct lining up codes, else joystick controll
      if (rightJoyStick.getRawButton(3) && mid.get()) {
        if (left.get())
          chassis.tankDrive(0.3, 0.2);
        else if (right.get())
          chassis.tankDrive(0.2, 0.3);
        else
          chassis.tankDrive(0.25, 0.25);
      } else {
        chassis.tankDrive(leftJoyStick.getY(), rightJoyStick.getY());
      }
    }
  }
}