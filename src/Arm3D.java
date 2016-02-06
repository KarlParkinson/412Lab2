import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;

public class Arm3D {

	EV3LargeRegulatedMotor j1;
	EV3LargeRegulatedMotor j2;
	EV3LargeRegulatedMotor j3;
	
	double l1;
	double l2;
	double l3;
	
	public Arm3D() {
		this.j1 = new EV3LargeRegulatedMotor(MotorPort.C);
		this.j2 = new EV3LargeRegulatedMotor(MotorPort.B);
		this.j2 = new EV3LargeRegulatedMotor(MotorPort.D);
		
		j1.setSpeed(30);
		j2.setSpeed(30);
		j3.setSpeed(30);
		
		
		this.l1 = 11;
		this.l2 = 13.5;
		this.l3 = 7.5;
		
		
	}
	
	public double[][] invKinematics(double x, double y, double z){
		double[][] angles = {{0},{0},{0}};
		
		
		return angles;
	}
	

}
