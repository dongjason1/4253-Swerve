
package org.usfirst.frc.team4253.robot;


import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;

import org.usfirst.frc.team4253.robot.Swerve;

public class Robot extends SampleRobot {
    Joystick stick = new Joystick(0);
    Swerve swerve;
    CANTalon motors[] = new CANTalon[8];
    AHRS ahrs;
    
    public void robotInit() {
        for(int i=0; i<8; i++){
        	motors[i] = new CANTalon(i);
        }
        
    	ahrs = new AHRS(SPI.Port.kMXP);
    	ahrs.reset();
		swerve = new Swerve(motors[0], motors[1], motors[2], motors[3], motors[4], motors[5], motors[6], motors[7], stick, ahrs);
    }

    public void operatorControl() {
    	swerve.resetEncoders();
        while (isOperatorControl() && isEnabled()) {
        	if(stick.getRawButton(12)) {
        		ahrs.reset();
        		swerve.resetEncoders();
        	}
        	swerve.swerveControl();
            Timer.delay(0.005);		// wait for a motor update time
        }
    }

}
