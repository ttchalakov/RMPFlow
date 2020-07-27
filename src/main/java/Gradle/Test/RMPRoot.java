package Gradle.Test;

import org.ejml.simple.SimpleMatrix;

//Probably move to inner class
public class RMPRoot extends RMPNode{
	public RMPRoot(String name)
	{
		super(name, null, null, null, null);
	}
	
	public void setRootState(SimpleMatrix x, SimpleMatrix x_dot)
	{
		if(x.numRows() == 1)
			this.x = x.transpose();
		else
			this.x = x;
		if(x_dot.numRows() == 1)
			this.x_dot = x_dot.transpose();
		else
			this.x_dot = x_dot;
//		if x.ndim == 1:
//			x = x.reshape(-1, 1)
//		if x_dot.ndim == 1:
//			x_dot = x_dot.reshape(-1, 1)
		
//		self.x = x
//		self.x_dot = x_dot
	}
	
	@Override
	public void pushforward()
	{
		for(int i = 0; i < getChildren().size(); i++)
			getChildren().get(i).pushforward();
	}
	
	public SimpleMatrix resolve()
	{
		System.out.println("A\n"+ m.toString());
		System.out.println(f.toString());
		a = m.pseudoInverse().mult(f);
		//self.a = np.dot(np.linalg.pinv(self.m), self.f);
		System.out.println(a.toString());
		return a;
	}
	
	public SimpleMatrix solve(SimpleMatrix x, SimpleMatrix x_dot)
	{
		setRootState(x, x_dot);
		pushforward();
		pullback();
		return resolve();
	}
}
