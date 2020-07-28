package Gradle.Test;

import org.ejml.simple.SimpleMatrix;

public class RMPFlowTester {

	public static void main(String[] args) {
		//Testing transpose
//		SimpleMatrix a = new SimpleMatrix(1, 2, false, new double[] {1, 2});
//		System.out.println(a.toString());
//		SimpleMatrix b = a.transpose();
//		System.out.println(b.toString());
//		
//		SimpleMatrix c = new SimpleMatrix(1, 1, false, new double[] {2});
//		SimpleMatrix d = new SimpleMatrix(1, 1, false, new double[] {3});
//		System.out.println("C * D = \n" + c.mult(d).toString());
		
		//RMP_example
		SimpleMatrix xyGoal = new SimpleMatrix(1, 2, false, new double[] {-2.5, 0});
		RMPRoot r = new RMPRoot("root");
		CollisionAvoidance collisionAvoidance = new CollisionAvoidance("Collision Avoidance Test"
				, r, new SimpleMatrix(1, 2), 1, .2, 1e-5, 0);
		GoalAttractor goalAttractor = new GoalAttractor("Goal Attractor Test", r, xyGoal, 10, 1, 1, 1, 2, 1, .005);
		
		SimpleMatrix x = new SimpleMatrix(1, 2, false, new double[] {8, -3.2});
		SimpleMatrix x_dot = new SimpleMatrix(1, 2, false, new double[] {-1, 2});
		
		
		SimpleMatrix x_ddot = r.solve(x, x_dot);
		//System.out.println(x_ddot.toString());
		
		//Lambda example
		//EvaluatableFunction<String> function = (a) -> a + "B";
		//System.out.println(function.of("No words end in "));
		/*
		# build the rmp tree
		x_g = np.array([-2.5, 0])
		x_o = np.array([0, 0])
		r_o = 1

		r = RMPRoot('root')
		leaf1 = CollisionAvoidance('collision_avoidance', r, None, epsilon=.1)
		leaf2 = GoalAttractorUni('goal_attractor', r, x_g)
		
				x = np.array([-8, -3.2])
				x_dot = np.array([-1, 2])
				
						state_0 = np.concatenate((x, x_dot), axis=None)
						
						# dynamics
						def dynamics(t, state):
						    state = state.reshape(2, -1)
						    x = state[0]
						    x_dot = state[1]
						    x_ddot = r.solve(x, x_dot)
						    state_dot = np.concatenate((x_dot, x_ddot), axis=None)
						    return state_dot
						    		
						    		sol = solve_ivp(dynamics, [0, 40], state_0)
						    		*/
	}

}
