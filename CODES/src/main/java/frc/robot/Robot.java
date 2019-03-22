/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.net.ServerSocket;
import java.net.Socket;
import java.util.Scanner;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
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

  private Compressor compressor;

  private DoubleSolenoid backsol, intake, frntsol;

  // False if not attached, will error if set true but cam not attached
  private boolean useCamera = true;
  private double liftHeight = 0.0d;

  public Robot() {
  }

  @Override
  public void robotInit() {
    // Ports according to design table
    chassis = new ChassisControl(2, 0, 1, 3);

    // The params for constructors might change due to design and such, change if
    // required
    SmartDashboard.putNumber("Height", 20.0d);
    liftEncoder = new Encoder(8, 7);
    leftE = new Encoder(0, 1);
    rightE = new Encoder(4, 3);
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
    compressor.setClosedLoopControl(false);
    backsol = new DoubleSolenoid(0, 1);
    intake = new DoubleSolenoid(2, 3);
    frntsol = new DoubleSolenoid(4, 5);
    if (useCamera) {
      camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setBrightness(0);
      camera.setExposureAuto();
      camera.setResolution(384, 288);
      camera.setFPS(120);
      camera.setWhiteBalanceAuto();

    }
  }

  /*
   * // Auton code here, used Init because only run once
   * 
   * @Override public void autonomousInit() { }
   */
  // These are the macros that can be used in either auton or teleop

  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }

  private boolean contClawIn = false;

  private void encoderForward(double distance, double baseSpeed) {

    leftE.reset();
    rightE.reset();
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
      chassis.tankDrive(-baseSpeed + kin * kinscale, -baseSpeed - kin * kinscale);
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

  private PIDLib motorPid = new PIDLib(0.3, 0.001, 0.1, 0.01);
  private PIDLib motorPid2 = new PIDLib(0.03, 0.001, 0.1, 0.01);
  private PIDLib motorPid4 = new PIDLib(0.1, 0.001, 0.1, 0.01);
  private double output;

  private boolean cball = false;
  private boolean it = false;
  private double scale = 1.0d;

  // Teleop code here
  @Override
  public void teleopPeriodic() {
    // Info, nop region
    {
      SmartDashboard.putNumber("LH", liftHeight);
      SmartDashboard.putNumber("LE", liftEncoder.getDistance());
      SmartDashboard.putNumber("Left E", leftE.getDistance());
      SmartDashboard.putNumber("Right E", rightE.getDistance());
    }
    if (liftHeight > liftEncoder.getDistance() || it) {
      if (cball) {
        output = motorPid.doPID(liftHeight - liftEncoder.getDistance());
      } else {
        output = motorPid4.doPID(liftHeight - liftEncoder.getDistance());
      }
    } else
      output = motorPid2.doPID(liftHeight - liftEncoder.getDistance());
    // Op region
    {
      if (altController.getRawButton(3)) {
        liftHeight += 0.1d;
      }
      if (altController.getRawButton(2)) {
        liftHeight -= 0.2d;
      }
      if (altController.getRawButton(7)) {
        compressor.setClosedLoopControl(false);
      } else if (altController.getRawButton(8)) {
        compressor.setClosedLoopControl(true);
      }
      if (altController.getRawButton(1)) {
        liftHeight = -7d;
        liftEncoder.reset();
      }
      if (altController.getRawButton(5)) {
        liftHeight = SmartDashboard.getNumber("Height", 3.0);
        ;
      } else if (altController.getRawButton(6)) {
        liftHeight = 0.0d;
      } else {

        liftMotor.set(-output);
      }
      if (rightJoyStick.getRawButton(4)) {
        intake.set(DoubleSolenoid.Value.kForward);
      } else if (rightJoyStick.getRawButton(5)) {
        intake.set(DoubleSolenoid.Value.kReverse);
      }
      if (leftJoyStick.getRawButton(5)) {
        claw.set(.7);
        claw2.set(.7);
      } else if (leftJoyStick.getRawButton(4)) {
        claw.set(-.7);
        claw2.set(-.7);
      } else {
        if (contClawIn) {
          claw.set(0.2);
          claw2.set(0.2);
        } else {
          claw.set(0.00);
          claw2.set(0.00);
        }
      }
      if (altController.getRawAxis(1) >= 0.5) {
        contClawIn = true;
      }
      if (altController.getRawAxis(1) <= -0.5) {
        contClawIn = false;
      }
      if (rightJoyStick.getRawButton(3)) {
        cball = false;
        liftHeight = 23.0d;
        // double sde = motorPid3.doPID(340 - receivefrompi);
        // chassis.tankDrive(0.1 - sde, 0.1 + sde);
      } // else {

      chassis.tankDrive(-leftJoyStick.getY() * scale * (-leftJoyStick.getZ() + 1.0) / 2.0,
          -rightJoyStick.getY() * scale * (-rightJoyStick.getZ() + 1.0) / 2.0);
      // }
      if (rightJoyStick.getRawButton(2)) {
        cball = false;
        liftHeight = 21.3d;
      }
      if (leftJoyStick.getRawButton(1)) {
        cball = true;
        liftHeight = 8d;
      }
      if (leftJoyStick.getRawButton(2)) {

        cball = true;
        liftHeight = 14.5d;
      }
      if (leftJoyStick.getRawButton(3)) {
        cball = true;
        liftHeight = 22.0d;
      }
      if (rightJoyStick.getRawButton(11)) {
        frntsol.set(DoubleSolenoid.Value.kForward);
      }
      if (rightJoyStick.getRawButton(10)) {

        frntsol.set(DoubleSolenoid.Value.kReverse);
      }
      if (altController.getRawAxis(2) > 0.5) {

        backsol.set(DoubleSolenoid.Value.kForward);
      }
      if (altController.getRawAxis(3) > 0.5) {

        backsol.set(DoubleSolenoid.Value.kReverse);
      }
      if (rightJoyStick.getRawButton(1)) {
        cball = false;
        liftHeight = 14.5d;
      }
      if (leftJoyStick.getRawButton(8)) {
        Timer.delay(0.5);

        encoderForward(1000, 0.4);
      }
      if (altController.getRawButton(4)) {
        intake.set(DoubleSolenoid.Value.kForward);
        it = true;
        liftHeight = liftEncoder.getDistance() + 0.1d;
        Thread t3 = new Thread() {
          public void run() {

            Timer.delay(0.5);
            it = false;

            encoderBackward(-200, 0.4);
          }
        };
        t3.start();
      }
      // TODO camera code
      if (altController.getRawButton(9)) {
        scale = 1.0d;
      }
      if (altController.getRawButton(10)) {
        scale = 1.0d / 2.0d;
      }
    }

  }
}