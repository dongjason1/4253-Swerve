package org.usfirst.frc.team4253.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;


public class Swerve {
	private CANTalon drive[] = new CANTalon[4];
	private CANTalon turn[] = new CANTalon[4];
	private Joystick stick;
	private AHRS ahrs1;

	private static float[] encodValues = new float[4];//these are the encoder values for the wheel's pivot direction
	private static boolean[] reversed = new boolean[4];//these determine if each wheel needs to be reveresed. used to solve the 180 problem
	private double angle = 0;//gyro value
	private double xJoy = 0;//this is the strafe value as decided by the joysticks
	private double yJoy = 0;//this is the move forwards value as decided by the joysticks
	private double rotation = 0;//this is the rotation value decided by the joysticks
	private boolean notMoving;//this is an override so that when the joysticks are close to 0 they don't move
	private boolean fieldOrientate;//decides if field oriented or not
	private final static double TICKSTODEGREES = 360/1700;
	
	public Swerve(CANTalon wheelsDirection0, CANTalon wheelsDirection1, CANTalon wheelsDirection2, CANTalon wheelsDirection3,
			CANTalon wheelsSpeed0, CANTalon wheelsSpeed1, CANTalon wheelsSpeed2, CANTalon wheelsSpeed3,Joystick joy, AHRS ahrs) {

		turn[0] = wheelsDirection0;turn[1] = wheelsDirection1;turn[2] = wheelsDirection2;turn[3] = wheelsDirection3;
		drive[0] = wheelsSpeed0;drive[1] = wheelsSpeed1;drive[2] = wheelsSpeed2;drive[3] = wheelsSpeed3;
		stick = joy;
		ahrs1 = ahrs;

		for (int i = 0; i < 4; i++) {
			drive[i].changeControlMode(TalonControlMode.PercentVbus);
			drive[i].setVoltageRampRate(40);
			drive[i].enableBrakeMode(true);
			drive[i].reverseOutput(false);//might need to reverse some of them depending on how they get set up

			
			turn[i].changeControlMode(TalonControlMode.PercentVbus);
			turn[i].setFeedbackDevice(FeedbackDevice.QuadEncoder);
			turn[i].enableBrakeMode(true);

			turn[i].reverseOutput(false);//also might need to be reversed 
			turn[i].reverseSensor(true);
		}
		
	}

	public void encodersUpdate() {//gets all the encoder values and maps them from 0-360
		for (int i = 0; i < 4; i++) {
			encodValues[i] = (float) (-turn[i].getEncPosition() * TICKSTODEGREES);//this ratio needs to be figured out
		}
		for (int i = 0; i < 4; i++) {
			while (encodValues[i] > 360)
				encodValues[i] -= 360;
			while (encodValues[i] < 0)
				encodValues[i] += 360;
		}
}

	public void joystickUpdate() {
		if (stick.getRawButton(5))//field oriented decision not on a toggle because i'm lazy
			fieldOrientate = true;
		else if (stick.getRawButton(7))
			fieldOrientate = false;

		if (fieldOrientate) {//this is the math for turning the inputs into a field oriented input
			xJoy = stick.getRawAxis(0) * Math.cos(angle) - stick.getRawAxis(0) * Math.sin(angle);
			yJoy = stick.getRawAxis(1) * Math.cos(angle) + stick.getRawAxis(1) * Math.sin(angle);
		} else {//normal inputs
			xJoy = stick.getRawAxis(0);
			yJoy = stick.getRawAxis(1);
		}

		rotation = stick.getRawAxis(2);
	}

	void decideIfNotMoving() {
		if (Math.abs(xJoy) < 0.1 && Math.abs(yJoy) < 0.1 && Math.abs(stick.getRawAxis(2)) < 0.1)
			notMoving = true;	
		else
			notMoving = false;
	}

	public double calculateSwerveDirection(int wheelNum) {//wheels 1,2,3,4 go from front right wheel and goes counter clockwise
		double a = xJoy-rotation*0.707;//math stuff. check the chief delphi paper for more info
		double b = xJoy+rotation*0.707;//it's just adding angular velocity and direction velocity
		double c = yJoy-rotation*0.707;//take it as a black box if you haven't studies physics yet
		double d = yJoy + rotation * 0.707;
		double x = 0;

		if (wheelNum == 1)x = Math.toDegrees(Math.atan2(b, c));
		if (wheelNum == 2)x = Math.toDegrees(Math.atan2(b, d));
		if (wheelNum == 3)x = Math.toDegrees(Math.atan2(a, d));
		if (wheelNum == 4)x = Math.toDegrees(Math.atan2(a, c));
		
		if (reversed[wheelNum - 1])//reverses direction if 180 problem is called
			x += 180;

		if (x > 360)//yep don't go above 360 it's really nasty
			x -= 360;

		return x;
	}
	
	public double calculateDirectionSpeed(double encoder, double direction, int wheelNum){
		double wheelsDirectionSpeed;
		if(direction>encoder){//if the target direction is greater than the current direction
			wheelsDirectionSpeed=(direction-encoder);//pivot towards the target direction at a scaling speed
			if(wheelsDirectionSpeed>180){//if the difference is greater than 180, it's faster to move in the negative direction
				//example of this encoder = 0, direction = 270, it's faster to move negative than positive
				wheelsDirectionSpeed-=360;//subtracting 360 properly changes the orientation
			}
		}
		else if(direction<encoder){//if the target direction is less than the current direction
			wheelsDirectionSpeed=-(encoder-direction);//same solution for the other one just in the other direction
			if(wheelsDirectionSpeed<-180) wheelsDirectionSpeed+=360;
		}
		else wheelsDirectionSpeed=0;
		if(notMoving)wheelsDirectionSpeed=0;//super override

		if(Math.abs(wheelsDirectionSpeed)>90){//sets the wheel to reversed if the direction is more than 90 degrees away(solves 180)
			reversed[wheelNum-1]=!reversed[wheelNum-1];
		}
		wheelsDirectionSpeed=wheelsDirectionSpeed/360;
		return wheelsDirectionSpeed;
	}
	
	double calculateSwerveSpeed(int wheelNum){//again this is math for each wheel speed.
		double a = xJoy-rotation*0.707;//go study physics
		double b = xJoy+rotation*0.707;
		double c = yJoy-rotation*0.707;
		double d = yJoy+rotation*0.707;
		if(wheelNum==1){
			if(reversed[wheelNum-1])return -Math.sqrt(Math.pow(b,2)+Math.pow(c,2));//takes into account needing to be reversed
			return Math.sqrt(Math.pow(b,2)+Math.pow(c,2));
		}
		if(wheelNum==2) {
			if(reversed[wheelNum-1])return -Math.sqrt(Math.pow(b,2)+Math.pow(d,2));
			return Math.sqrt(Math.pow(b,2)+Math.pow(d,2));
		}
		if(wheelNum==3) {
			if(reversed[wheelNum-1])return -Math.sqrt(Math.pow(a,2)+Math.pow(d,2));
			return Math.sqrt(Math.pow(a,2)+Math.pow(d,2));
		}
		if(wheelNum==4) {
			if(reversed[wheelNum-1])return -Math.sqrt(Math.pow(a,2)+Math.pow(c,2));
			return Math.sqrt(Math.pow(a,2)+Math.pow(c,2));
		}
		return 0;
	}
	
	void resetEncoders(){//self explanatory
		for (int i = 0; i < 4; i++) {
			turn[i].setEncPosition(0);
		}
	}
	
	void gyroUpdate(){//vex gyros are stupid. NavX won't need this
		angle = ahrs1.getAngle();
	}
	
	void swerve(){
		turn[0].set(calculateDirectionSpeed(encodValues[0],calculateSwerveDirection(1),1));
		turn[1].set(calculateDirectionSpeed(encodValues[1],calculateSwerveDirection(2),2));
		turn[2].set(calculateDirectionSpeed(encodValues[2],calculateSwerveDirection(3),3));
		turn[3].set(calculateDirectionSpeed(encodValues[3],calculateSwerveDirection(4),4));

		drive[0].set(calculateSwerveSpeed(2));
		drive[1].set(calculateSwerveSpeed(3));
		drive[2].set(calculateSwerveSpeed(1));
		drive[3].set(calculateSwerveSpeed(4));
	}
	
	void updateSmartDashboard(){
		for(int i = 0; i<4; i++){
			SmartDashboard.putNumber("Encoder " + i, turn[i].getEncPosition());
		}
		SmartDashboard.putNumber("NavX", ahrs1.getAngle());
	}
	
	void swerveControl(){
		gyroUpdate();
		encodersUpdate();
		joystickUpdate();
		decideIfNotMoving();
		updateSmartDashboard();
		swerve();
	}
}
