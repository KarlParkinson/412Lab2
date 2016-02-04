import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

import java.lang.Math;

import lejos.utility.Matrix;;


public class Arm {
	
	EV3LargeRegulatedMotor j1;
	EV3LargeRegulatedMotor j2;
	
	double l1;
	double l2;
	
	public Arm() {
		this.j1 = new EV3LargeRegulatedMotor(MotorPort.D);
		this.j2 = new EV3LargeRegulatedMotor(MotorPort.A);
		
		j1.setSpeed(30);
		j2.setSpeed(30);
		

		this.l2 = 8.7;
		this.l1 = 11.2;
		
		
	}
	
	public void goToAngle(double theta1, double theta2) {
		
		
		
		this.j1.rotateTo((int)theta1, true);
		this.j2.rotateTo((int)theta2);
		
		
		/*moved to new function
		double radians1= theta1 * Math.PI/180;
		double radians2 = theta2 * Math.PI/180;
		double x = this.l1*Math.cos(radians1) + this.l2*Math.cos(radians1 + radians2);
		double y = this.l1*Math.sin(radians1) + this.l2*Math.sin(radians1 + radians2);
		 */
		forwardKinematics(theta1,theta2);
	
		Button.waitForAnyPress();
	}
	
	public double[] forwardKinematics(double theta1, double theta2){
		double radians1= theta1 * Math.PI/180;
		double radians2 = theta2 * Math.PI/180;
		double x = this.l1*Math.cos(radians1) + this.l2*Math.cos(radians1 + radians2);
		double y = this.l1*Math.sin(radians1) + this.l2*Math.sin(radians1 + radians2);
		
		//System.out.printf("x: %.2f \n",x);
		//System.out.printf("y: %.2f \n",y);
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
		
		System.out.printf("p1: (%.1f,%.1f) \n",pos1[0],pos1[1]);
		System.out.printf("p2: (%.1f,%.1f) \n",pos2[0],pos2[1]);
		System.out.printf("distance: %.1f \n",distance);
		
		Button.waitForAnyPress();
	}
	
	public double[][] invKinematics(double x, double y){
		double[][] angles = {{j1.getTachoCount()},{j2.getTachoCount()}};
		double[] pos = {x,y};
		Matrix angleMat = new Matrix(angles);
		double Jx1,Jx2,Jy1,Jy2;

		
		//J.print(System.out);
		//Button.waitForAnyPress();
		for(int i = 0; i < 15;i++){
			
			Jx1 = -1*this.l1*Math.sin(angleMat.get(0,0)) - this.l2*Math.sin(angleMat.get(0,0) + angleMat.get(1, 0));
			Jx2 = -1*this.l2*Math.sin(angleMat.get(0,0) + angleMat.get(1, 0));
			
			Jy1 = this.l1*Math.cos(angleMat.get(0,0)) - this.l2*Math.cos(angleMat.get(0,0) + angleMat.get(1, 0));
			Jy2 = this.l2*Math.cos(angleMat.get(0,0) + angleMat.get(1, 0));
			
			//System.out.println(x);
			
			double[][] array = {{Jx1,Jx2},{Jy1,Jy2}};
			Matrix J = new Matrix(array);
			
			Matrix s = J.solve(f(angleMat, pos));
			angleMat.plusEquals(s);
		}
		angles = angleMat.getArrayCopy();
		
		angles[0][0] = angles[0][0] % (2*Math.PI);
		angles[1][0] = angles[1][0] % (2*Math.PI);
		
		
		return angles;
	}
	
	public double[][] invKinematics2(double x, double y){
		double[][] angles = {{0},{0}};
		
		double D = (Math.pow(x, 2) + Math.pow(y, 2) - Math.pow(this.l1, 2) - Math.pow(this.l2, 2)) / (2 * this.l1*this.l2);
		
		double z = Math.sqrt(1-Math.pow(D, 2))/D;
		
		double theta2 = Math.atan2(-z,z);
		
		double theta1 =Math.atan(y/x) - Math.atan(this.l2 * Math.sin(theta2) /(this.l1 + this.l2*Math.cos(theta2)));
		
		angles[0][0] = theta1;
		angles[1][0] = theta2;
		
		return angles;
	}
	
	public void gotToPoint(double x, double y){
		double[][] angles = invKinematics(x,y);
		
		System.out.printf("theta1: %.2f \n",Math.toDegrees(angles[0][0]));
		System.out.printf("theta2: %.2f \n",Math.toDegrees(angles[1][0]));

		this.goToAngle(Math.toDegrees(angles[0][0]), Math.toDegrees(angles[1][0]));
		
	}
	
	
	public Matrix f(Matrix angles, double[] pos){
		double fx = this.l1*Math.cos(angles.get(0, 0)) + this.l2*Math.cos(angles.get(0, 0) + angles.get(1, 0)) - pos[0];
		double fy = this.l1*Math.sin(angles.get(0, 0)) + this.l2*Math.sin(angles.get(0, 0) + angles.get(1, 0)) - pos[1];
		double[][] f = {{-fx},{-fy}};
		
		return new Matrix(f);
	}
	
	public void measureAngle(){
		double[] pos1, pos2, pos3 = {0,0};
		double distance, m1, m2;
		
		//this.j1.rotateTo(0, true);
		//this.j2.rotateTo(0);
		
		System.out.println(j1.getTachoCount());
		System.out.println(j2.getTachoCount());
		
		System.out.println("Pick intersection \n and press button");
		pos1 = getPoint();
		System.out.println("Pick 1st line \n and press button");
		pos2 = getPoint();
		System.out.println("Pick 2nd line \n and press button");
		pos3 = getPoint();
		
		double u[] = {pos2[0] - pos1[0],pos2[1] - pos1[1]};
		double v[] = {pos3[0] - pos1[0],pos3[1] - pos1[1]};
		
		double ul = Math.sqrt(Math.pow(u[0],2) + Math.pow(u[1], 2));
		double vl = Math.sqrt(Math.pow(v[0],2) + Math.pow(v[1], 2));
		
		double angle = Math.acos(((u[0]*v[0])+u[1]*v[1])/(ul*vl));	
		
		System.out.printf("p1: (%.1f,%.1f) \n",pos1[0],pos1[1]);
		System.out.printf("p2: (%.1f,%.1f) \n",pos2[0],pos2[1]);
		System.out.printf("p3: (%.1f,%.1f) \n",pos3[0],pos3[1]);
		System.out.printf("angle: %.1f \n",Math.toDegrees(angle));
		
		
		Button.waitForAnyPress();
		
	}
	
	public void findMidPoint(){
		double[] pos1, pos2 = {0,0};
		double distance;
		
		System.out.println("Pick point \n and press button");
		pos1 = getPoint();
		System.out.println("Pick 2nd point \n and press button");
		pos2 = getPoint();
		
		
		double x = (pos1[0] + pos2[0])/2;
		double y = (pos1[1] + pos2[1])/2;
		
		this.gotToPoint(x, y);		
		
		
		System.out.printf("p1: (%.1f,%.1f) \n",pos1[0],pos1[1]);
		System.out.printf("p2: (%.1f,%.1f) \n",pos2[0],pos2[1]);
		System.out.printf("mid: (%.1f,%.1f) \n",x,y);
		
		Button.waitForAnyPress();
	}
	
	
	
	public static void main(String[] args) {
		Arm a = new Arm();
		//a.goToAngle(180, 270);
		a.measureDistance();
		//a.measureAngle();
		//a.gotToPoint(1, 10);
		//a.findMidPoint();
		/*
		double[][] angles1, angles2 = {{},{}};
		
		double x = 6;
		double y = 6;
		
		angles1 = a.invKinematics(x, y);
		angles2 = a.invKinematics2(x, y);
		
		System.out.println();
		*/
		
		//a.gotToPoint(, );;
	}
	
	
	

}
