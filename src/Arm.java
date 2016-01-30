import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
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
		this.l1 = 11.5;
		
		
	}
	
	public void goToAngle(int theta1, int theta2) {
		this.j1.rotateTo(theta1, true);
		this.j2.rotateTo(theta2);
		
		
		/*moved to new function
		double radians1= theta1 * Math.PI/180;
		double radians2 = theta2 * Math.PI/180;
		double x = this.l1*Math.cos(radians1) + this.l2*Math.cos(radians1 + radians2);
		double y = this.l1*Math.sin(radians1) + this.l2*Math.sin(radians1 + radians2);
		 */
		forwardKinematics(theta1,theta2);
	
		Button.waitForAnyPress();
	}
	
	public double[] forwardKinematics(int theta1, int theta2){
		double radians1= theta1 * Math.PI/180;
		double radians2 = theta2 * Math.PI/180;
		double x = this.l1*Math.cos(radians1) + this.l2*Math.cos(radians1 + radians2);
		double y = this.l1*Math.sin(radians1) + this.l2*Math.sin(radians1 + radians2);
		
		System.out.println(x);
		System.out.println(y);
		double[] pos = {x,y};
		return pos;
	}
	
	/*
	 * Waits for button to be pressed and returns the point the 
	 * arm end effector is currently at .
	 */
	public double[] getPoint(){
		double[] point = {0,0};
		
		this.j1.flt();
		this.j2.flt();
		Button.waitForAnyPress();
		point = forwardKinematics(j1.getTachoCount(),j2.getTachoCount());
		
		return point;
	}
	
	/*
	 * Measures the distance between two points selected by moving 
	 * arm and pressing EV3 brick button
	 */
	public void measureDistance(){
		double[] pos1, pos2 = {0,0};
		double distance;
		
		//this.j1.rotateTo(90, true);
		//this.j2.rotateTo(0);
		
		System.out.println("Pick point \n and press button");
		pos1 = getPoint();
		System.out.println("Pick 2nd point \n and press button");
		pos2 = getPoint();
		
		// d = sqrt(y2-y1)^2 + (x2-x1)^2)
		distance = Math.sqrt(Math.pow((pos2[0]-pos1[0]),2) + Math.pow((pos2[1]-pos1[1]),2));
		System.out.println("Distance: " + distance);
		Button.waitForAnyPress();
	}
	
	public void measureAngle(){
		double[] pos1, pos2, pos3 = {0,0};
		double distance, slope1, slope2;
		
		this.j1.rotateTo(0, true);
		this.j2.rotateTo(0);
		
		System.out.println("Pick intersection \n and press button");
		pos1 = getPoint();
		System.out.println("Pick 1st line \n and press button");
		pos2 = getPoint();
		System.out.println("Pick 2nd line \n and press button");
		pos3 = getPoint();
		
		slope1 = Math.abs(pos1[1] - pos2[1]) / Math.abs(pos1[0] - pos2[0]);
		slope2 = Math.abs(pos1[1] - pos3[1]) / Math.abs(pos1[0] - pos3[0]);
		
		double angle = Math.atan((slope1 -slope2) / (1+ slope1*slope2));
		
		System.out.println("Angle: " + Math.toDegrees(angle));
		Button.waitForAnyPress();
		
	}
	
	
	public static void main(String[] args) {
		Arm a = new Arm();
		//a.goToAngle(180, 270);
		a.measureDistance();
		//a.measureAngle();
	}
	
	
	

}
