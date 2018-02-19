/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2408.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import java.math.*;

import com.kauailabs.navx.frc.AHRS;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot implements PIDOutput {
	private static final int posGround = 0;
	private static final int posSwitch = 45;
	private static final int posScale = 90;
	
	private static final String RightStart = "Right Start";
	private static final String LeftStart = "Left Start";
	private static final String MiddleStart = "Middle Start";
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private static double armP = 0.0;
	private static double armI = 0.0;
	private static double armD = 0.0;
	
	private static double driveP = 0.0;
	private static double driveI = 0.0;
	private static double driveD = 0.0;
	
	private static double angleP = 0.0;
	private static double angleI = 0.0;
	private static double angleD = 0.0;
	
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	private Timer m_timer = new Timer();
	private PIDController armPID;
	private PIDController frontLeftPID;
	private PIDController backLeftPID;
	private PIDController frontRightPID;
	private PIDController backRightPID;
	private PIDController navXPID;
	
	public AHRS ahrs;

	//Speed Controller Initialization 
	//Drive Motors
	Spark frontRight = new Spark(0);
	Spark backRight = new Spark(1);
	Spark frontLeft = new Spark(2);
	Spark backLeft = new Spark(3);
	//Intake Motors
	Spark shootLeft = new Spark(4);
	Spark shootRight = new Spark(5);
	//Arm Motor
	Spark armMotor = new Spark(6);
	
	//Encoders
	//Drive System
	Encoder rightEnc = new Encoder(0, 1);
	Encoder leftEnc = new Encoder(2,3);
	//Arm System
	Encoder armEnc = new Encoder(4,5);
	
	//Pneumatics Control Module
	Compressor pow = new Compressor(0);
	//Intake Mechanism
	DoubleSolenoid grabber = new DoubleSolenoid(0, 1);
	DoubleSolenoid intake = new DoubleSolenoid(2, 3);
	
	//Joystick Initialization
	//Driver 1
	Joystick rightJoystick = new Joystick(0);
	Joystick leftJoystick = new Joystick(1);
	//Driver 2
	Joystick utility = new Joystick(2);
	
	@Override
	public void robotInit() {
		//Autonomous Selection
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		m_chooser.addObject("Right Line", RightStart);
		m_chooser.addObject("Left Line", LeftStart);
		m_chooser.addObject("Middle Start", MiddleStart);
		SmartDashboard.putData("Auto choices", m_chooser);
		
		//Motor Direction
		frontLeft.setInverted(true);
		backLeft.setInverted(true);
		
		//USB Camera Initialization
		CameraServer.getInstance().startAutomaticCapture();
		
		//Cylinder Position
		intake.set(Value.kReverse);
		grabber.set(Value.kReverse);
		armPID = new PIDController(armP, armI, armD, armEnc, armMotor);
		armPID.setOutputRange(-.7, .7);
		
		//NavX Initialization via USB
		try {
        ahrs = new AHRS(SerialPort.Port.kUSB);
		} catch (RuntimeException ex ) {
        DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}
		
		frontLeftPID = new PIDController(driveP, driveI, driveD, leftEnc, frontLeft);
		frontLeftPID.setOutputRange(-1, 1);
		backLeftPID = new PIDController(driveP, driveI, driveD, leftEnc, backLeft);
		backLeftPID.setOutputRange(-1, 1);
		frontRightPID = new PIDController(driveP, driveI, driveD, rightEnc, frontRight);
		frontRightPID.setOutputRange(-1, 1);
		backRightPID = new PIDController(driveP, driveI, driveD, rightEnc, backRight);
		backRightPID.setOutputRange(-1, 1);
		
		navXPID = new PIDController(angleP, angleI, angleD, ahrs, this);
		navXPID.setOutputRange(-1, 1);
		navXPID.setInputRange(-180, 180);
		navXPID.setContinuous(true);
		
		
		ahrs.reset();
		SmartDashboard.putString("OUR SWITCH IS ", isOursLeft(0)? "LEFT":"RIGHT");
		SmartDashboard.putString("OUR SCALE IS ", isOursLeft(1)? "LEFT":"RIGHT");
		SmartDashboard.putString("THEIR SWITCH IS ", isOursLeft(2)? "LEFT":"RIGHT");
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);
		
		//Autonomous Timer Initialization
		m_timer.reset();
		m_timer.start();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		switch (m_autoSelected) {
			case LeftStart:
				if(isOursLeft(0)) {
					driveStraight(133);
					armCommand(posSwitch);
					turnSystem(90);
					driveStraight(37.5);
					grabRelease(true);
				} else if(isOursLeft(1)) {
					driveStraight(318);
					armCommand(posScale);
					turnSystem(90);
					driveStraight(10);
					grabRelease(true);
				} else {
					driveStraight(150);
				}
				break;
			case MiddleStart:
				if(isOursLeft(0)) {
					driveStraight(50);
					turnSystem(-90);
					driveStraight(131);
					turnSystem(0);
					driveStraight(94);
					armCommand(posSwitch);
					turnSystem(90);
					driveStraight(10);
				} else {
					driveStraight(50);
					turnSystem(90);
					driveStraight(60);
					turnSystem(0);
					armCommand(posSwitch);
					driveStraight(55);
					turnSystem(-90);
					driveStraight(10);
					grabRelease(true);
				}
				
				
				
				
				break;
			case RightStart:
				if(!isOursLeft(0)) {
					driveStraight(133);
					armCommand(posSwitch);
					turnSystem(-90);
					driveStraight(37.5);
					grabRelease(true);
				} else if(!isOursLeft(1)) {
					driveStraight(318);
					armCommand(posScale);
					turnSystem(-90);
					driveStraight(10);
					grabRelease(true);
				} else {
					driveStraight(150);
				}
				break;
			case kCustomAuto:
				System.out.println("MEMES");
				System.out.println("ARE DREAMS");
				break;
			case kDefaultAuto:
			default:
				System.out.println("MEMES");
				System.out.println("ARE DREAMS");
				break;
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		
		//Local variable name for the Left Y axis and Right Y axis
		double rightY = rightJoystick.getY();
		double leftY = leftJoystick.getY();
		
		//Data output for encoders
		SmartDashboard.putNumber("Right", rightEnc.getRaw());
		SmartDashboard.putNumber("Left", leftEnc.getRaw());
		SmartDashboard.putNumber("Arm", armEnc.getRaw());
		
		//Intake Motor Protocol
		if(utility.getRawButton(8)) {
			shootLeft.set(1);
			shootRight.set(1);
		} else if(utility.getRawButton(9)) {
			shootLeft.set(-1);
			shootRight.set(-1);
		} else {
			shootLeft.set(0);
			shootRight.set(0);
		}
		//Intake Pistons
		if(utility.getRawButton(3)) {
			intake.set(Value.kForward);
		} else if(utility.getRawButton(2)) {
			intake.set(Value.kReverse);
		}
		
		//Grabber Pistons
		if(utility.getRawButton(10)) {
			grabber.set(Value.kForward);
		} else if(utility.getRawButton(11)) {
			grabber.set(Value.kReverse);
		}
		
		//Tank Drive
		frontRight.set(rightY);
		backRight.set(rightY);
		frontLeft.set(leftY);
		backLeft.set(leftY);
		
		//Arm Logic
		if(utility.getPOV() == 180) {
			armCommand(posScale);
		} else if (utility.getPOV() == 90) {
			armCommand(posSwitch);
		} else if (utility.getPOV() == 0) {
			armCommand(posGround);
		}
		
		
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	
	public void armCommand(int armAngle) {
		double SetpointAngle = ((1024.0 *5 )/360) * armAngle;
		armPID.setSetpoint(SetpointAngle);
	}
	
	public void driveStraight(double Distance) {
		frontRightPID.enable();
		frontLeftPID.enable();
		backRightPID.enable();
		backLeftPID.enable();
		rightEnc.reset();
		leftEnc.reset();
		double pulsePerInch = 1440.0 /(Math.PI * 6);
		double driveSetpoint = pulsePerInch * Distance;
		frontRightPID.setSetpoint(driveSetpoint);
		backRightPID.setSetpoint(driveSetpoint);
		frontLeftPID.setSetpoint(driveSetpoint);
		backLeftPID.setSetpoint(driveSetpoint);
		while (Math.abs(frontLeftPID.getError()) > 5 && Math.abs(frontRightPID.getError()) > 5) {
			Timer.delay(.015);
		}
		frontRightPID.disable();
		frontLeftPID.disable();
		backRightPID.disable();
		backLeftPID.disable();
	}
	
	public void turnSystem(double fieldDeg) {
		navXPID.enable();
		navXPID.setSetpoint(fieldDeg);
		while (Math.abs(navXPID.getError()) > 5) {
			Timer.delay(.015);
		}
		navXPID.disable();
	}
	
	public void grabRelease(boolean isReleased) {
		if(isReleased) {
			grabber.set(Value.kForward);
			Timer.delay(.5);
		} else {
			grabber.set(Value.kReverse);
			Timer.delay(.5);
		}
	}
	
	public boolean isOursLeft(int dexOne) {
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		if(gameData.charAt(dexOne) == 'L') {
			return true;
		} else {
			return false;
		}
	}

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		frontRight.set(output);
		backRight.set(output);
		frontLeft.set(output);
		backLeft.set(output);
	}
	
}
