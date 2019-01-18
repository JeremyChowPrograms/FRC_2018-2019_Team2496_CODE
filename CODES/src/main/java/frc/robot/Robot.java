/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  private final int kJoystickPort = 0;
  private final int kJoystickPort2 = 1;
  private Joystick leftJoyStick,rightJoyStick;
  private ChassisControl chassis;
  private UsbCamera camera;
  private boolean isTeleop = false;
  public Robot(){
    chassis = new ChassisControl(0, 1, 2, 3);
  }
  @Override
  public void robotInit() {  
    leftJoyStick = new Joystick(kJoystickPort);
    rightJoyStick = new Joystick(kJoystickPort2);
    camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(800,600);
    camera.setFPS(60);
  }

  //Auton code here, used Init because only run once
  @Override
  public void autonomousInit() {
  }

  //Init Code
  @Override
  public void teleopInit() {
      isTeleop=this.isOperatorControl();
      //New Thread
      {
      Thread t = new Thread(){
          @Override
          public void run(){
            while(isTeleop){
  
            }
          }
      };
      t.start();}
  }

  //Teleop code here
  @Override
  public void teleopPeriodic() {
    chassis.tankDrive(leftJoyStick.getY(), rightJoyStick.getY());
    isTeleop=this.isOperatorControl();
  }
}