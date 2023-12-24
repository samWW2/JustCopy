// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.xml.transform.Source;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.SPI; // this import





//change the name of the class to "DriveSubsystem"
//pay attention that the class extends SubsystemBase
public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax leftMotor1 = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax leftMotor2 = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax rightMotor1 = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax rightMotor2 = new CANSparkMax(4, MotorType.kBrushless);
  private final double kDegToMeters = 0.076 * Math.PI ;
  private final MotorControllerGroup m_left = new MotorControllerGroup(leftMotor1, leftMotor2);
  private final MotorControllerGroup m_right = new MotorControllerGroup(rightMotor1, rightMotor2);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_left, m_right);
  private RelativeEncoder rightEncoder = rightMotor1.getEncoder();
  private RelativeEncoder leftEncoder = leftMotor1.getEncoder();
  private double factor = (((8.45 * 15.24)/10000))*4.6;
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final double start_angel = 148;
  



  //similar to the init call
  public DriveSubsystem() {
    rightMotor1 .setInverted(true);
    rightMotor2.setInverted(true);
    rightEncoder.setPositionConversionFactor(factor);
    leftEncoder.setPositionConversionFactor(factor);
  }

  

  public void setIdleMode(IdleMode mode) {
    leftMotor1.setIdleMode(mode);
  }


  @Override
  public void periodic() {
  }
  public void setMotors(double leftSpeed, double rightSpeed) {
    m_left.set(leftSpeed);
    m_right.set(rightSpeed);
    
}
  
  public void resetGyro(){
    gyro.reset();
  }  

  public CANSparkMax getRightMotor1(){
    return this.rightMotor1;
  }
  public CANSparkMax getLeftMotor1(){
    return this.leftMotor1;
  }
  
  public void arcadeDrive(double str, double turn){
   
    m_robotDrive.arcadeDrive(str, turn);
  }
  public void tankDrive(double left, double right){
    m_robotDrive.tankDrive(left, right);
  }
  public void resetEncoders(){
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }
public boolean isYawChanged(){
  if(Math.abs(getYaw()) > 15){   
     return true;
   }
   return false;
}
  public double getYaw(){
    double angle = gyro.getYaw() + start_angel;
    if(angle > 180){
      angle -= 360;
    }
    return angle;
  }
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  public double getRightEncoder(){
    return rightEncoder.getPosition();
  }
  public double getleftEncoder(){
    return leftEncoder.getPosition();
  }
  public double getBothEncoders(){
    return (getRightEncoder() + getleftEncoder())/2;
  }

  
}