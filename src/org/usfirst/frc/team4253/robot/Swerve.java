package org.usfirst.frc.team4253.robot;

import edu.wpi.first.wpilibj.Joystick;
import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import com.ctre.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;


public class Swerve {
	private CANTalon drive[] = new CANTalon[4];
	private CANTalon turn[] = new CANTalon[4];
	private Joystick stick;
	private AHRS ahrs1;
	
	private static float[] encodValues = new float[4];//these are the encoder values for the wheel's pivot direction
	private static boolean[] reversed =  new boolean[4];//these determine if each wheel needs to be reveresed. used to solve the 180 problem
	private double angle = 0;//gyro value
	private double xJoy = 0;//this is the strafe value as decided by the joysticks
	private double yJoy = 0;//this is the move forwards value as decided by the joysticks
	private double rotation = 0;//this is the rotation value decided by the joysticks
	private boolean notMoving;//this is an override so that when the joysticks are close to 0 they don't move
	private boolean fieldOrientate;//decides if field oriented or not
	
	private final static int encoderTicksPerRevolution = 256;
	private final static double F = 0.5f;//change these
	private final static double P = 1f;
	private final static double I = 0.000001f;
	private final static double D = 1.0f;
	
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

			
			turn[i].changeControlMode(TalonControlMode.Position);
			turn[i].setFeedbackDevice(FeedbackDevice.AnalogEncoder);
			turn[i].enableBrakeMode(true);
			turn[i].reverseOutput(false);//also might need to be reversed 
			turn[i].reverseSensor(false);
			turn[i].setPID(P, I, D, F, 0, 12, 0);
			turn[i].configNominalOutputVoltage(+0.0f, -0.0f);
			turn[i].configPeakOutputVoltage(+12.0f, -12.0f);
			
		}
	}

	public void encodersUpdate() {//gets all the encoder values and maps them from 0-360
		for (int i = 0; i < 4; i++) {
			encodValues[i] = ((float)(turn[i].getAnalogInPosition()-20)/2.7777f);//this ratio needs to be figured out
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
		if (Math.abs(xJoy) < 0.01 && Math.abs(yJoy) < 0.01 && Math.abs(stick.getRawAxis(2)) < 0.01)
			notMoving = true;	
		else
			notMoving = false;
	}

	public double calculateSwerveDirection(int wheelNum) {//wheels 1,2,3,4 go from front right wheel and goes counter clockwise
		double a = xJoy - rotation * 0.707;//math stuff. check the chief delphi paper for more info
		double b = xJoy + rotation * 0.707;//it's just adding angular velocity and direction velocity
		double c = yJoy - rotation * 0.707;//take it as a black box if you haven't studies physics yet
		double d = yJoy + rotation * 0.707;//returns an angle from 0-360
		double x = 0;

		if (wheelNum == 1) x = Math.toDegrees(Math.atan2(b, c));
		if (wheelNum == 2) x = Math.toDegrees(Math.atan2(b, d));
		if (wheelNum == 3) x = Math.toDegrees(Math.atan2(a, d));
		if (wheelNum == 4) x = Math.toDegrees(Math.atan2(a, c));

		if (reversed[wheelNum - 1])//reverses direction if 180 problem is called
			x += 180;

		if (x > 360)//yep don't go above 360 it's really nasty
			x -= 360;

		return x;
	}

	public double calculateDirection(double encoder, double direction, int wheelNum){
		//this will take an angle from 0-360, map it to the encoder ticks per revolution, and find target PID position
		//also will know if the 180 thing needs to happen
		//returns an int that is the encoder tick it needs to go to
		if(notMoving) return encoder;
		
		double targetPosition=direction*(encoderTicksPerRevolution/360.0);//maps the 360 value to the number of encoder ticks
		while(Math.abs(encoder-targetPosition)>(encoderTicksPerRevolution/2.0)){//this will get the position closest to the current encoder value
			if(encoder>targetPosition)targetPosition+=encoderTicksPerRevolution;
			else targetPosition-=encoderTicksPerRevolution;
		}
		
		if(Math.abs(encoder-targetPosition)>(encoderTicksPerRevolution/4.0)){//if this is true it needs to be reversed
			reversed[wheelNum-1]=!reversed[wheelNum-1];
			return encoder;//return this for this pass so that it doesn't spaz and will properly reverse
		}
		return targetPosition;
	}
	
	double calculateSwerveSpeed(int wheelNum){//again this is math for each wheel speed.
		double a = xJoy-rotation*0.707;//go study physics
		double b = xJoy+rotation*0.707;//should return a double from -1.0 to 1.0
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
			turn[i].setAnalogPosition(0);
			turn[i].setAnalogPosition(0);
			turn[i].setAnalogPosition(0);
		}
	}
	
	void gyroUpdate(){//vex gyros are stupid. NavX won't need this
		angle = ahrs1.getAngle();
	}
	
	void swerve(){
		//wheels 1,2,3,4 go from front right wheel and goes counter clockwise
		turn[0].set(calculateDirection(encodValues[0],calculateSwerveDirection(2),2));//turn[0] is back right
		turn[1].set(calculateDirection(encodValues[1],calculateSwerveDirection(3),3));//turn[1] is back left
		turn[2].set(calculateDirection(encodValues[2],calculateSwerveDirection(1),1));//turn[2] is front right
		turn[3].set(calculateDirection(encodValues[3],calculateSwerveDirection(4),4));//turn[3] is front left

		drive[0].set(calculateSwerveSpeed(2));//drive[0] is back right
		drive[1].set(calculateSwerveSpeed(3));//drive[1] is back left
		drive[2].set(calculateSwerveSpeed(1));//drive[2] is front right
		drive[3].set(calculateSwerveSpeed(4));//drive[3] is front left
	}
	void swerveControl(){
		SmartDashboard.putNumber("Turn 0", calculateDirection(encodValues[0],calculateSwerveDirection(2),2));
		SmartDashboard.putNumber("Navx", ahrs1.getAngle());
		SmartDashboard.putNumber("Encoder0", turn[0].getAnalogInRaw());
		SmartDashboard.putNumber("Encoder1", turn[1].getAnalogInRaw());
		SmartDashboard.putNumber("Encoder2", turn[2].getAnalogInRaw());
		SmartDashboard.putNumber("Encoder3", turn[3].getAnalogInRaw());
		gyroUpdate();
		encodersUpdate();
		joystickUpdate();
		decideIfNotMoving();
		swerve();
	}
}
