import java.awt.geom.Point2D;

public class RVector {

    Point2D m_vec;

    public RVector (Point2D begin, Point2D end) {
        m_vec = new Point2D.Double ();        
        m_vec.setLocation (
            (end.getX () - begin.getX ()),
            (end.getY () - begin.getY ())); 
    }
    
    /* inner product: this . b */
    public double dot (RVector b) {
        return
            getVecX () * b.getVecX () + 
            getVecY () * b.getVecY ();  
    }
    
    public double perp (RVector b) {
        return 
            getVecX () * b.getVecY () - 
            getVecY () * b.getVecX ();
    }

    public double getVecX () {
        return m_vec.getX ();
    }
    
    public double getVecY () {
        return m_vec.getY ();
    }
}