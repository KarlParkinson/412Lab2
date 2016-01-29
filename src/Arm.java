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
	
	
	public void measureDistance(){
		double[] pos1, pos2 = {0,0};
		double distance;
		
		System.out.println("Pick 1st point \n and press button");
		//this.j1.rotateTo(0, true);
		//this.j2.rotateTo(0);
		this.j1.flt();
		this.j2.flt();
		Button.waitForAnyPress();
		pos1 = forwardKinematics(j1.getTachoCount(),j2.getTachoCount());
		System.out.println("Pick 2nd point \n and press button");
		this.j1.flt();
		this.j2.flt();
		Button.waitForAnyPress();
		pos2 = forwardKinematics(j1.getTachoCount(),j2.getTachoCount());
		
		distance = Math.sqrt(Math.pow((pos2[0]-pos1[0]),2) + Math.pow((pos2[1]-pos1[1]),2));
		System.out.println("Distance: " + distance);
		Button.waitForAnyPress();
	}
	
	
	public static void main(String[] args) {
		Arm a = new Arm();
		//a.goToAngle(180, 270);
		
	}
	
	
	

}
