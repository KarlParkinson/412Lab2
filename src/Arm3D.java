import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
/*
 * 3DOF arm class
 */
public class Arm3D {

	EV3LargeRegulatedMotor j1; //first motor (closest)
	EV3LargeRegulatedMotor j2; //second motor
	EV3LargeRegulatedMotor j3; //third motor (farthest) 
	
	double link1;
	double link2;
	double link3;
	
	public Arm3D() {
		this.j1 = new EV3LargeRegulatedMotor(MotorPort.B);
		this.j2 = new EV3LargeRegulatedMotor(MotorPort.C);
		this.j3 = new EV3LargeRegulatedMotor(MotorPort.A);
		
		j1.setSpeed(50);
		j2.setSpeed(50);
		j3.setSpeed(50);
		
		this.link1 = 12.7;
		this.link2 = 13.5;
		this.link3 = 7.5;
	}
	
	/*
	 * Analytic solution to 3DOF arm 
	 */
	public double[][] invKinematics(double x, double y, double z){
		double[][] angles = {{0},{0},{0}};
		double theta1,theta2,theta3;
		double l1,l2;
		
		//calculate angle needed for z height
		theta3 = Math.asin(z/link3);
		
		l1 = link1;
		//l2 is the length of link two plus the horizontal distance of link3 at angle theta3
		l2 = link2 + link3*Math.cos(theta3);
		
		double D = (Math.pow(x, 2) + Math.pow(y, 2) - Math.pow(l1, 2) - Math.pow(l2, 2)) / (2 * l1*l2);
		
		double tmp = Math.sqrt(1-Math.pow(D, 2))/D;
		
		theta2 = Math.atan(tmp);
		
		theta1 =Math.atan(y/x) - Math.atan(l2 * Math.sin(theta2) /(l1 + l2*Math.cos(theta2)));
		
		angles[0][0] = theta1;
		angles[1][0] = theta2;
		angles[2][0] = theta3;
		
		return angles;
	}
	
	/*
	 * Uses inverse kinematics to find angles needed to reach point x,y,z
	 */
	public void goToPoint(double x, double y, double z){
		double[][] angles = invKinematics(x,y,z);
		
		System.out.printf("theta1: %.2f \n",Math.toDegrees(angles[0][0]));
		System.out.printf("theta2: %.2f \n",Math.toDegrees(angles[1][0]));
		System.out.printf("theta3: %.2f \n",Math.toDegrees(angles[2][0]));

		double t1 = Math.toDegrees(angles[0][0]);
		double t2 = Math.toDegrees(angles[1][0]);
		double t3 = Math.toDegrees(angles[2][0]);
		
		this.j1.rotateTo((int)t1, true);
		this.j2.rotateTo((int)t2);
		this.j3.rotateTo((int)-t3);
	}
	
	/*
	 * Function for moving a box located at (x,y,z) to (x2,y2,z2)
	 */
	public void moveBox(double x, double y, double z, double x2, double y2) {
		this.goToPoint(x, y, z);
		//Button.waitForAnyPress();
		this.goToPoint(x, y, z-3);
		//Button.waitForAnyPress();
		this.goToPoint(x2, y2, z-3);
	}

	
	public static void main(String[] args) {
		
		Arm3D a = new Arm3D();
		
		//a.goToPoint(25, -5, 5);
		//Button.waitForAnyPress();
		a.moveBox(25, -5, 0, 15, 11);
		//Button.waitForAnyPress();
	}
}
