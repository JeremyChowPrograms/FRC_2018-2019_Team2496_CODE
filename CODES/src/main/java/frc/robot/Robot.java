/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final int kJoystickPort = 0;
  private final int kJoystickPort2 = 1;
  private Joystick leftJoyStick, rightJoyStick;
  private ChassisControl chassis;
  private UsbCamera camera;
  private Encoder liftEncoder;
  private SpeedController liftMotor;
  private SpeedController claw;
  private boolean isTeleop = false;
  private Compressor compressor;
  private DigitalLib left, mid, right;

  private DoubleSolenoid lift1, lift2, intake;

  private boolean useCamera = false;
  private double liftHeight = 0.0d;

  public Robot() {
    chassis = new ChassisControl(3, 4, 1, 2);// Changed temporarily, normal is 0,1,2,3
  }

  @Override
  public void robotInit() {
    liftEncoder = new Encoder(0, 1);
    liftEncoder.setDistancePerPulse(Math.PI / 200);
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
  }

  // Init Code
  @Override
  public void teleopInit() {
    isTeleop = this.isOperatorControl();
    {
      Thread t = new Thread() {
        @Override
        public void run() {
          PIDLib pid = new PIDLib(0.3, 0, 0.03, 0.0);
          while (isTeleop) {
            liftMotor.set(pid.PID(liftEncoder.getDistance(), liftHeight));
          }
        }
      };
      t.start();
    }
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
    lift1.set(leftJoyStick.getRawButton(2) ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    lift2.set(leftJoyStick.getRawButton(3) ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    intake.set(leftJoyStick.getRawButton(4) ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    // Macro
    {
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