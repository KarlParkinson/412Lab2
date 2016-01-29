import lejos.hardware.port.MotorPort;
import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

import java.lang.Math;

public class Arm {
	
	EV3LargeRegulatedMotor j1;
	EV3LargeRegulatedMotor j2;
	
	double l1;
	double l2;
	
	public Arm() {
		this.j1 = new EV3LargeRegulatedMotor(MotorPort.D);
		this.j2 = new EV3LargeRegulatedMotor(MotorPort.A);
		
		this.l1 = 4.8;
		this.l2 = 6.4;
		
	}
	
	public void goToAngle(int theta1, int theta2) {
		this.j1.rotateTo(theta1, true);
		this.j2.rotateTo(theta2);
		
		double radians1= theta1 * Math.PI/180;
		double radians2 = theta2 * Math.PI/180;
		double x = this.l1*Math.cos(radians1) + this.l2*Math.cos(radians1 + radians2);
		double y = this.l1*Math.sin(radians1) + this.l2*Math.sin(radians1 + radians2);
		
		System.out.println(x);
		System.out.println(y);
		Button.waitForAnyPress();
	}
	
	public static void main(String[] args) {
		Arm a = new Arm();
		a.goToAngle(180, 270);
	}
	
	
	

}
