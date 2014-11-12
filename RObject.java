import java.util.Iterator;
import java.util.ArrayList;
import java.awt.geom.Point2D;

public class RObject {

    /* data points */
    ArrayList<Point2D> m_points;
    ArrayList<Point2D> m_expandedPoints;
    
    public RObject () {
        m_points = new ArrayList<Point2D> ();
        m_expandedPoints = new ArrayList<Point2D> ();
    }

    public void setPoint (double x, double y) {
        m_points.add (new Point2D.Double (x, y));
    }
    
    public void setExpPoint (double x, double y) {
        m_expandedPoints.add (new Point2D.Double (x, y));
    }
    
    public int numPoints () {
        return m_points.size ();
    }
    
    public Point2D getPointIdx (int idx) {
        return m_points.get (idx);
    }
    
    public void printObject () {

        System.out.println ("Object Dump:");

        Iterator<Point2D> it = m_points.iterator();

        while (it.hasNext ()) {
            Point2D obj = it.next ();
            System.out.println ("\t" + obj.toString ());
        }
        
        System.out.println ("");
    }
}