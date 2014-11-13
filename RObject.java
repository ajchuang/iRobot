import java.util.Iterator;
import java.util.ArrayList;
import java.util.Vector;
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
    
    public Point2D calculateCenterOfMass () {
    
        double x = 0.0, y = 0.0;
        
        Iterator<Point2D> it = m_points.iterator();
        
        while (it.hasNext ()) {
            Point2D obj = it.next ();
            
            x += obj.getX ();
            y += obj.getY ();
        }
        
        /* calculate center of mass */
        Point2D com = new Point2D.Double ();
        com.setLocation (x / m_points.size(), y / m_points.size());
        return com;
    }
    
    /* TODO: complex function @@" */
    public void expandMargin () {
        Point2D com = calculateCenterOfMass ();
        
        /* For all polygons,        */
        /* For each node,           */
        /* 1.   calculate K and B   */
        /* 2.   derive delta-B      */
        /* 2.5  save the equation   */
        /* 3.   calculate new (x,y) */
    
    }
    
    public Vector<Point2D> getExpandedVertex () {
        
        Vector<Point2D> v = new Vector<Point2D> ();
        
        /* temp solution: we have not expanded yet */
        Iterator<Point2D> it = m_points.iterator();
        
        /*
        Iterator<Point2D> it = m_expandedPoints.iterator();
        */
        
        while (it.hasNext ()) {
            Point2D obj = it.next ();
            v.add ((Point2D)obj.clone ());
        }
        
        return v;
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