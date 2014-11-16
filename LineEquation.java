



public class LineEquation {

	double k; /* slope */
	double b; /* interception of y */
	boolean k_flag; /* 1 if k is infinity */ 
	double xb; /* interception of x */

	public LineEquation(double k_value, double b_value, boolean k_flag_value, double xb_value){
		k = k_value;
		b = b_value;
		k_flag = k_flag_value;
		xb = xb_value;
	}

	public LineEquation(){
		k = 0.0;
		b = 0.0;
		k_flag = false;
		xb = 0.0;
	}

	public double getK(){
		return k;
	}

	public double getB(){
		return b;
	}

	public boolean getKFLAG(){
		return k_flag;
	}

	public double getXB(){
		return xb;
	}
}