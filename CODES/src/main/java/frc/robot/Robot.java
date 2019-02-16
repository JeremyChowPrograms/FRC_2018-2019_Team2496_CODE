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
  private Joystick leftJoyStick, rightJoyStick, altController;
  private ChassisControl chassis;
  private UsbCamera camera;
  private Encoder liftEncoder, leftE, rightE;
  private SpeedController liftMotor;
  private SpeedController claw, claw2;

  private ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  private Compressor compressor;

  private DoubleSolenoid liftsol, intake;

  // False if not attached, will error if set true but cam not attached
  private boolean useCamera = true;
  private double liftHeight = 0.0d;

  public Robot() {
  }

  @Override
  public void robotInit() {
    // Ports according to design table
    chassis = new ChassisControl(2, 0, 1, 3);

    gyro.calibrate();
    gyro.reset();

    // The params for constructors might change due to design and such, change if
    // required
    liftEncoder = new Encoder(8, 7);
    leftE = new Encoder(0, 1, 2);
    rightE = new Encoder(3, 4, 5);
    liftEncoder.setDistancePerPulse(Math.PI / 200);
    leftE.setDistancePerPulse(6 * Math.PI / 200);
    rightE.setDistancePerPulse(6 * Math.PI / 200);
    leftJoyStick = new Joystick(0);
    rightJoyStick = new Joystick(1);
    altController = new Joystick(2);
    claw = new Talon(6);
    claw2 = new Talon(7);
    liftMotor = new Talon(5);
    compressor = new Compressor();
    compressor.setClosedLoopControl(true);
    liftsol = new DoubleSolenoid(0, 1);
    intake = new DoubleSolenoid(2, 3);
    if (useCamera) {
      camera = CameraServer.getInstance().startAutomaticCapture(1);
      camera.setResolution(400, 300);
      camera.setFPS(120);
    }
  }

  // Auton code here, used Init because only run once
  @Override
  public void autonomousInit() {
    // encoderForward(72, 0.4);
  }

  // These are the macros that can be used in either auton or teleop
  private void leftTurn(double degrees, double speed) {
    gyro.reset();
    double doPid = 0;
    PIDLib pid = new PIDLib(0.008f, 0, 0.00001, 0);
    while (degrees + gyro.getAngle() > 5) {
      doPid = pid.doPID(-degrees - 5 - gyro.getAngle());
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
      doPid = pid.doPID(degrees + 5 - gyro.getAngle());
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

  @Override
  public void teleopInit() {
    liftEncoder.reset();
    leftE.reset();
    rightE.reset();
  }

  private PIDLib motorPid = new PIDLib(0.5, 0.001, 0.1, 0.01);
  private double output;

  // Teleop code here
  @Override
  public void teleopPeriodic() {
    // Info, nop region
    {
      SmartDashboard.setDefaultNumber("Value", leftJoyStick.getZ());
    }
    output = motorPid.doPID(liftHeight - liftEncoder.getDistance());
    // Op region
    {

      if (altController.getRawButton(7)) {
        compressor.setClosedLoopControl(false);
      } else if (altController.getRawButton(8)) {
        compressor.setClosedLoopControl(true);
      }
      if (leftJoyStick.getRawButton(8)) {
        leftTurn(90, 0.4);
      } else if (leftJoyStick.getRawButton(9)) {
        rightTurn(90, 0.4);
      }
      if (altController.getRawButton(1)) {
        liftHeight = 0.0d;
        liftEncoder.reset();
      }
      if (altController.getRawButton(5)) {
        liftMotor.set(-0.5);
      } else if (altController.getRawButton(6)) {
        liftMotor.set(0.3);
      } else {

        // liftMotor.set(-output);
      }
      if (rightJoyStick.getRawButton(4)) {
        intake.set(DoubleSolenoid.Value.kForward);
      } else if (rightJoyStick.getRawButton(5)) {
        intake.set(DoubleSolenoid.Value.kReverse);
      }
      if (leftJoyStick.getRawButton(4)) {
        claw.set(1);
        claw2.set(1);
      } else if (leftJoyStick.getRawButton(5)) {
        claw.set(-1);
        claw2.set(-1);
      }
      // TODO camera code
      if (rightJoyStick.getRawButton(3)) {

      } else {
        chassis.tankDrive(-leftJoyStick.getY() * (-leftJoyStick.getZ() + 1.0) / 2.0,
            -rightJoyStick.getY() * (-rightJoyStick.getZ() + 1.0) / 2.0);
      }
    }
  }
}