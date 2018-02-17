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
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	private static final String RightLine = "Right Line";
	private static final String LeftLine = "Left Line";
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	private Timer m_timer = new Timer();

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
	Encoder RightEnc = new Encoder(0, 1);
	Encoder LeftEnc = new Encoder(2,3);
	//Arm System
	Encoder ArmEnc = new Encoder(4,5);
	
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
		m_chooser.addObject("Right Line", RightLine);
		m_chooser.addObject("Left Line", LeftLine);
		SmartDashboard.putData("Auto choices", m_chooser);
		
		//Motor Direction
		frontLeft.setInverted(true);
		backLeft.setInverted(true);
		
		//USB Camera Initialization
		CameraServer.getInstance().startAutomaticCapture();
		
		//Cylinder Position
		intake.set(Value.kReverse);
		grabber.set(Value.kReverse);
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
			case LeftLine:
				if(m_timer.get() < 4) {
					frontLeft.set(1);
					backLeft.set(1);
					frontRight.set(1);
					backRight.set(1);
				} else {
					frontLeft.stopMotor();
					backLeft.stopMotor();
					frontRight.stopMotor();
					backRight.stopMotor();
				}
			break;
			case RightLine:
				if(m_timer.get() < 4) {
					frontLeft.set(1);
					backLeft.set(1);
					frontRight.set(1);
					backRight.set(1);
				} else {
					frontLeft.stopMotor();
					backLeft.stopMotor();
					frontRight.stopMotor();
					backRight.stopMotor();
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
		SmartDashboard.putNumber("Right", RightEnc.getRaw());
		SmartDashboard.putNumber("Left", LeftEnc.getRaw());
		SmartDashboard.putNumber("Arm", ArmEnc.getRaw());
		
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
		
		//Tank Drive
		frontRight.set(rightY);
		backRight.set(rightY);
		frontLeft.set(leftY);
		backLeft.set(leftY);
		
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
