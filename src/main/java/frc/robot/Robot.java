// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cameraserver.CameraServer;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in th
 * e TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  //private final Drivetrain drivetrain = new Drivetrain();
  private RobotContainer m_robotContainer;
  Drivetrain drivetrain;
  Joystick joystick;
  Joystick arm_joystick;
  public static double jxArray[] = new double[Constants.MacroTime];
  public static double tAngleArray[] = new double[Constants.MacroTime];
  public static double dPowerArray[] = new double[Constants.MacroTime];
  public static boolean triggerArray[] = new boolean[Constants.MacroTime];
  public static boolean b2Array[] = new boolean[Constants.MacroTime];
  private int idx = Constants.MacroTime;
  //private double taDouble;
  //private double tvDouble;
  private double txDouble;
  private double tyDouble;
  private Arm arm;
  private double LRoll;
  private double cold;
  private double SYaw;
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_robotContainer=new RobotContainer();
    this.drivetrain = m_robotContainer.drivetrain;
    this.joystick = m_robotContainer.joystick;
    this.arm = m_robotContainer.arm;
    this.arm_joystick = m_robotContainer.arm_joystick;
    drivetrain.resetDEncoders();
    CameraServer.startAutomaticCapture(0);
    CameraServer.startAutomaticCapture(1);
    //cameraSelection=NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
//CameraServer.startAutomaticCapture();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    drivetrain.gyroPutPitch();
    drivetrain.gyroPutRoll();
    drivetrain.gyroPutYaw();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx"); // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    NetworkTableEntry ty = table.getEntry("ty"); // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
    //NetworkTableEntry tv = table.getEntry("tv"); // Whether the limelight has any valid targets (0 or 1)
    //NetworkTableEntry ta = table.getEntry("ta"); // Target Area (0% of image to 100% of image)
    //drivetrain.CANtest();
    //taDouble = ta.getDouble(0.0);
    //tvDouble = tv.getDouble(0.0);
    txDouble = tx.getDouble(0.0);
    tyDouble = ty.getDouble(0.0);

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    //if(SmartDashboard.getBoolean("CameraSel", true)){
    //  System.out.println("Setting Camera 2");
    //  cameraSelection.setString(camera2.getName());
   // }
    //else{
    //  System.out.println("Setting Camera 1");
    //  cameraSelection.setString(camera1.getName());
    //}
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {arm.set_servos(800,300,320,2000);}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    drivetrain.resetDEncoders();
    arm.set_servos(800,300,320,2000);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }
  public double getDistanceFromTopOfPeg() {
    double targetOffsetAngle_Vertical = tyDouble;
    double angleToGoalDegrees = Constants.angleLimeDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    double distanceFromLimelightToPegBaseInches = (Constants.heightTopInch - Constants.heightLimeInch)/Math.tan(angleToGoalRadians);
    return distanceFromLimelightToPegBaseInches;
  }
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}
  @Override
  public void teleopInit() {
    drivetrain.gyroSetYaw(0);
    //arm.set_servos(800,300,320,2000);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double jX;
    double jY;
    double tAngle;
    double dPower;
    boolean b2;
    boolean trigger;    
    jX=joystick.getX();
    jY=joystick.getY()*-1;
    b2=joystick.getRawButton(2);
    trigger=joystick.getTrigger();
    double[] cTpResult = drivetrain.cTp(jX, jY);
    tAngle=cTpResult[0];
    dPower=cTpResult[1];
    if(joystick.getRawButton(7)){
      double e=drivetrain.gyroGetRoll()-LRoll;
      if(Math.abs(e)>Constants.ChargeStationConstants.climbDeg){cold=0;}
      double p = Constants.ChargeStationConstants.gain*e;
      p=(p>Constants.ChargeStationConstants.clip)?Constants.ChargeStationConstants.clip:((p<-Constants.ChargeStationConstants.clip)?-Constants.ChargeStationConstants.clip:p);
      if(cold==1) p=Constants.ChargeStationConstants.start;
      double lp; 
      double rp;
      lp = p+(Constants.ChargeStationConstants.lrgain*(drivetrain.gyroGetYaw()-SYaw));
      rp = p-(Constants.ChargeStationConstants.lrgain*(drivetrain.gyroGetYaw()-SYaw));
      if(cold==1){drivetrain.RobotLRMove(90, lp, rp);}
      else{
        if(p<0){drivetrain.RobotLRMove(270, -lp, -rp);}
        else{drivetrain.RobotLRMove(90, lp, rp);}}}
    else{
      if(joystick.getRawButton(8)){LRoll=drivetrain.gyroGetRoll();cold=1;SYaw=drivetrain.gyroGetYaw();}
      if(joystick.getRawButton(11)){idx=0;}
      if(idx<Constants.MacroTime){jxArray[idx]=jX;tAngleArray[idx]=tAngle;dPowerArray[idx]=dPower;triggerArray[idx]=trigger;b2Array[idx]=b2;idx++;System.out.println(idx);}
      if(joystick.getRawButton(4)){tAngle = (tAngle-(txDouble*Constants.LimeLightAimAssistG))%360;} //limelight aim assist
      drivetrain.RobotMove(tAngle, dPower, jX, trigger, b2);

    }
      //System.out.println(txDouble);
  }
  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
  
  /*public void setPipeline(Integer pipeline) {
    if (pipeline < 0) {
      pipeline = 0;
      throw new IllegalArgumentException("Pipeline can not be less than zero");
    } else if (pipeline > 9) {
      pipeline = 9;
      throw new IllegalArgumentException("Pipeline can not be greater than nine");
    }
    table.getEntry("pipeline").setValue(pipeline);
  }*/

}
