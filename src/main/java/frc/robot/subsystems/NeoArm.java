// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;
import frc.robot.Constants;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;




public class NeoArm extends Arm {
  
  

  CANSparkMax armMotor = new CANSparkMax(30,com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  RelativeEncoder armMotorEncoder = armMotor.getEncoder();
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  SparkMaxPIDController m_pidController;

  public NeoArm(Joystick arm_joystick) {
    super(arm_joystick);
    m_pidController = armMotor.getPIDController();
    // PID coefficients
 kP = 1.0; 
 kI = 1e-4;
 kD = 0.1; 
 kIz = 0; 
 kFF = 0; 
 kMaxOutput = 0.5; 
 kMinOutput = -0.5;


  // set PID coefficients
  m_pidController.setP(kP);
  m_pidController.setI(kI);
  m_pidController.setD(kD);
  m_pidController.setIZone(kIz);
  m_pidController.setFF(kFF);
  m_pidController.setOutputRange(kMinOutput, kMaxOutput);
  }
  public void periodic() {
    super.periodic();
  
}
public void moveShoulder(double numTurns) {
    //Set the arm to zero position
    armMotorEncoder.setPosition(0.0);
    //Rotate the motor 20 times
    m_pidController.setReference(-20, CANSparkMax.ControlType.kPosition);

}
public void resetShoulder() {
  armMotor.restoreFactoryDefaults();
  armMotor.setIdleMode(IdleMode.kBrake);
  //Set the arm to zero position
  armMotorEncoder.setPosition(0.0);
  //Slow the motor down
  armMotor.set(0.001);

 
}

@Override
public void set_servos(double claw_set, double wrist_set, double elbow_set, double shoulder_set ) {
  //System.out.print(claw_set);
  servo_W(s0, claw_set); //claw_init
  servo_W(s1, wrist_set); //wrist_init 
  servo_W(s2, elbow_set); //elbow joint 
  //servo_W(s3, shoulder_set); //shoulder_init
  moveShoulder(shoulder_set);
}

}
