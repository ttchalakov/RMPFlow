package Gradle.Test;

import org.ejml.simple.SimpleMatrix;

public class CollisionAvoidance extends RMPLeaf {
	private double r, alpha, eta, epsilon;
	
	public CollisionAvoidance(String name, RMPNode parent, SimpleMatrix center, double r, double epsilon, double alpha, double eta)
	{
		super(name, parent, null, null, null);
		this.r = r;
		this.alpha = alpha;
		this.eta = eta;
		this.epsilon = epsilon;
		
		System.out.println("center\n" + center.toString());
		
		if(center.numRows() == 1)
			center = center.transpose();
		
		System.out.println("center\n" + center.toString());
		
		int n = center.getNumElements();
		
		final SimpleMatrix finalCenter = center;
		//np.array(norm(y - c) / R - 1).reshape(-1,1)
		psi = (y) -> (new SimpleMatrix(1, 1, false
				, new double[] {y.minus(finalCenter).normF() / r - 1}).transpose());
		
		//1.0 / norm(y - c) * (y - c).T / R
		j = (y) ->
		{
			double scale = 1/y.minus(finalCenter).normF();
			return y.minus(finalCenter).transpose().divide(r).scale(scale);
		};
		
		//np.dot(
        //y_dot.T,
        //(-1 / norm(y - c) ** 3 * np.dot((y - c), (y - c).T)
        //    + 1 / norm(y - c) * np.eye(N))) / R
		j_dot = (SimpleMatrix y, SimpleMatrix y_dot) ->
		{
			SimpleMatrix a = y.minus(finalCenter).mult(y.minus(finalCenter).transpose())
					.scale(-1 / Math.pow(y.minus(finalCenter).normF(), 3));
			System.out.println("a test\n" + a.toString());
			//System.out.println("-1 / Math.pow(y.minus(finalCenter).normF(), 3)\n" + -1 / Math.pow(y.minus(finalCenter).normF(), 3));
			SimpleMatrix b = a.plus(eye(n).scale(1 / y.minus(finalCenter).normF()));
			System.out.println("b test\n" + b.toString());
			System.out.println("eye(n).scale(1 / y.minus(finalCenter).normF())\n" + eye(n).scale(1 / y.minus(finalCenter).normF())
					+ "\ny\n" + y.toString()
					+ "\nfinalCenter\n" + finalCenter.toString()
					+ "\ny.minus(finalCenter)\n" + y.minus(finalCenter)
					+ "\ny.minus(finalCenter).normF()\n" + y.minus(finalCenter).normF()
					+ "\n1 / y.minus(finalCenter).normF()\n" + 1 / y.minus(finalCenter).normF()
					+ "\neye(n)\n" + eye(n).toString());
			//System.out.println("\n1 / y.minus(finalCenter).normF()\n" + 1 / y.minus(finalCenter).normF());
			return y_dot.transpose().mult(b).divide(this.r);
		};
	}
	
	public void solveF()
	{
		System.out.println("Solving for F Collision Avoidance");
		System.out.println("x\n" + x.toString() + "\nx_dot\n" + x_dot.toString());
		double w;
		double grad_w;
		if(x.get(0, 0) < 0)
		{
			w = 1e10;
			grad_w = 0;
		}
		else
		{
			w = 1 / Math.pow(x.get(0, 0), 4);
			grad_w = -4 / Math.pow(x.get(0, 0), 5);
		}
		System.out.println("w\n" + w + "\ngrad_w\n" + grad_w);
		
		double u = epsilon + Math.min(0, x_dot.get(0, 0)) * x_dot.get(0, 0);
		double g = w * u;
		System.out.println("u\n" + u + "\ng\n" + g);
		
		double grad_u = 2 * Math.min(0, x_dot.get(0, 0));
		double grad_Phi = alpha * w * grad_w;
		double xi = .5 * Math.pow(x_dot.get(0, 0), 2) * u * grad_w;
		System.out.println("grad_u\n" + grad_u + "\ngrad_Phi\n" + grad_Phi + "\nxi\n" + xi);
		
//		double m_double = g + .5 * x_dot.get(0, 0) * w * grad_u;
//		this.m = new SimpleMatrix(1, 1, false, new double[] {Math.min(Math.max(-1e5, m_double), 1e5)});
		
		double bx_dot = eta * g * x_dot.get(0, 0);
		System.out.println("bx_dot" + bx_dot);
		
		double f_double = -grad_Phi - xi - bx_dot;
		this.f = new SimpleMatrix(1, 1, false, new double[] {Math.min(Math.max(-1e10, f_double), 1e10)});
		
		System.out.println("F\n" + f.toString());
	}
	
	public void solveM()
	{
		System.out.println("Solving for M Collision Avoidance");
		System.out.println("x\n" + x.toString() + "\nx_dot\n" + x_dot.toString());
		double w;
		double grad_w;
		if(x.get(0, 0) < 0)
		{
			w = 1e10;
			grad_w = 0;
		}
		else
		{
			w = 1 / Math.pow(x.get(0, 0), 4);
			grad_w = -4 / Math.pow(x.get(0, 0), 5);
		}
		System.out.println("w\n" + w + "\ngrad_w\n" + grad_w);
		
		double u = epsilon + Math.min(0, x_dot.get(0, 0)) * x_dot.get(0, 0);
		double g = w * u;
		System.out.println("u\n" + u + "\ng\n" + g);
		
		double grad_u = 2 * Math.min(0, x_dot.get(0, 0));
		double grad_Phi = alpha * w * grad_w;
		double xi = .5 * Math.pow(x_dot.get(0, 0), 2) * u * grad_w;
		System.out.println("grad_u\n" + grad_u + "\ngrad_Phi\n" + grad_Phi + "\nxi\n" + xi);
		
		double m_double = g + .5 * x_dot.get(0, 0) * w * grad_u;
		this.m = new SimpleMatrix(1, 1, false, new double[] {Math.min(Math.max(-1e5, m_double), 1e5)});
		
		System.out.println("M\n" + m.toString());
		
//		double Bx_dot = eta * g * x_dot.get(0, 0);
//		
//		double f_double = -grad_Phi - xi - Bx_dot;
//		this.f = new SimpleMatrix(1, 1, false, new double[] {Math.min(Math.max(-1e10, f_double), 1e10)});
	}
}