import java.util.Iterator;
import java.util.ArrayList;
import java.util.Vector;
import java.awt.geom.Point2D;
import java.lang.Math;

/* UI components */
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.geom.Line2D;

import javax.swing.JOptionPane;
import javax.swing.JComponent;
import javax.swing.SwingUtilities;

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
    
    public Point2D getExpPointIdx (int idx) {
        if (m_expandedPoints.size () == 0)
            return m_points.get (idx);
        else
            return m_expandedPoints.get (idx);
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
        double expand_distance = 0.175;
        ArrayList<LineEquation> edge_expanded; /* x for slope, y for interception and z as a flag if slope is infinity (in which case x stands for the interception of x axis instead of the slope) */
        edge_expanded = new ArrayList<LineEquation> ();
        /*while (it.next ().hasNext ()) {
            Point2D obj_1 = it.next ();
            Point2D obj_2 = it.next ().next ();
            m_expandedPoints.add ((Point2D)obj.clone ());
        }*/
        Point2D point_1, point_2, point_ref;

        double x_1, x_2, x_ref, y_1, y_2, y_ref, xb, xb_expanded, k, b, b_expanded;

        for (int i = 0; i < m_points.size(); i++){
            point_1 = m_points.get (i);
            if (i == m_points.size() - 1){
                point_2 = m_points.get (0);
                point_ref = calculateCenterOfMass ();
            }
            else {
                point_2 = m_points.get (i + 1);
                point_ref = calculateCenterOfMass ();
            }
        
        
            x_1 = point_1.getX ();
            y_1 = point_1.getY ();
            x_2 = point_2.getX ();
            y_2 = point_2.getY ();
            x_ref = point_ref.getX ();
            y_ref = point_ref.getY ();

            /* find the line by the two points, expand and store */
            if (Math.abs(x_1 - x_2) == 0) {
                xb = x_1;
                /* check the location of the reference point to decide the moving dircetion */
                if (x_ref > xb) {
                    xb_expanded = xb - expand_distance;
                }
                else {
                    xb_expanded = xb + expand_distance;
                }
                System.out.println("!!!XB: " + xb_expanded);
                edge_expanded.add (new LineEquation (0.0, 0.0, true, xb_expanded));
            }
            else {
                k = (y_1 - y_2) / (x_1 - x_2);
                b = (x_2 * y_1 - x_1 * y_2) / (x_2 - x_1);
                /* check the location of the reference point to decide the moving dircetion */
                if (y_ref > k * x_ref + b){
                    b_expanded = b - expand_distance * Math.sqrt (1 + k * k);
                }
                else {
                    b_expanded = b + expand_distance * Math.sqrt (1 + k * k);
                }
                System.out.println("!!!K AND B:" + k + " , " + b_expanded);
                edge_expanded.add (new LineEquation (k, b_expanded, false, 0.0));
            }
        }
        
        double xb_1, k_2, b_2, x_expanded, y_expanded, xb_2, k_1, b_1;
        LineEquation edge_1, edge_2;
        /* find the intersection points */
        for (int i = 0; i < edge_expanded.size(); i++){
            edge_1 = edge_expanded.get (i);
            if (i == edge_expanded.size() - 1){
                edge_2 = edge_expanded.get (0);
            }
            else {
                edge_2 = edge_expanded.get (i + 1);
            }

            if (edge_1.getKFLAG () == true){
                xb_1 = edge_1.getXB ();
                k_2 = edge_2.getK ();
                b_2 = edge_2.getB ();
                x_expanded = xb_1;
                y_expanded = k_2 * x_expanded + b_2;
            }
            else if (edge_2.getKFLAG () == true){
                xb_2 = edge_2.getXB ();
                k_1 = edge_1.getK ();
                b_1 = edge_1.getB ();
                x_expanded = xb_2;
                y_expanded = k_1 * x_expanded + b_1;
            }
            else {
                k_1 = edge_1.getK ();
                b_1 = edge_1.getB ();
                k_2 = edge_2.getK ();
                b_2 = edge_2.getB ();
                x_expanded = (b_2 - b_1) / (k_1 - k_2);
                y_expanded = k_1 * x_expanded + b_1;
            }
            /* store the expanded point */
            setExpPoint (x_expanded, y_expanded);
            System.out.println("******* EXPANDED POINTS: *******");
            System.out.println(x_expanded);
            System.out.println(y_expanded);
        }
        
        System.out.println ("Original Points: ");
        for (int i = 0; i < m_expandedPoints.size (); ++i)
            System.out.println ("\t" + m_points.get(i));
        
        System.out.println ("Expanded Points: ");
        for (int i = 0; i < m_expandedPoints.size (); ++i)
            System.out.println ("\t" + m_expandedPoints.get(i));
    }
    
    public Vector<Point2D> getExpandedVertex () {
        
        Vector<Point2D> v = new Vector<Point2D> ();
        
        /* temp solution: we have not expanded yet */
        Iterator<Point2D> it = m_expandedPoints.iterator();
        
        while (it.hasNext ()) {
            Point2D obj = it.next ();
            v.add ((Point2D)obj.clone ());
        }
        
        return v;
    }
    
    public void paint (Graphics g) {
        
        Point2D pt_0, pt_1;
        int n = m_points.size ();
        
        g.setColor (Color.black);
    
        for (int i = 0; i < n; ++i) {
            
            pt_0 = PathPlanner.transformCoord (m_points.get (i));
            pt_1 = PathPlanner.transformCoord (m_points.get ((i + 1) % n));
            
            System.out.println ("Drawing " + pt_0 + " to " + pt_1);
            g.drawLine (
                (int) pt_0.getX (),
                (int) pt_0.getY (),
                (int) pt_1.getX (),
                (int) pt_1.getY ()
            );
        } 
        
        g.setColor (Color.blue);
        
        if (m_expandedPoints.size() != n) {
            System.out.println ("Oooops");
            return;
        }
        
        for (int i = 0; i < n; ++i) {
            
            pt_0 = PathPlanner.transformCoord (m_expandedPoints.get (i));
            pt_1 = PathPlanner.transformCoord (m_expandedPoints.get ((i + 1) % n));
            
            g.drawLine (
                (int) pt_0.getX (),
                (int) pt_0.getY (),
                (int) pt_1.getX (),
                (int) pt_1.getY ()
            );
        }
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