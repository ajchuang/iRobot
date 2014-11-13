import java.util.Iterator;
import java.util.ArrayList;
import java.util.StringTokenizer;
import java.io.BufferedReader;
import java.io.FileReader;
import java.awt.geom.Point2D;

public class PathPlanner {
    
    /* constants */
    final double m_robotDiameter    = 35.0;
    final double m_robotRadius      = 35.0 / 2;
    
    /* data points */
    RObject m_workSpace;
    ArrayList<RObject> m_obj;

    /* 2 special points: start & end */
    Point2D.Double m_start;
    Point2D.Double m_end;
    
    public PathPlanner () {
        m_obj = new ArrayList<RObject> ();
    }
    
    public void createObjects (String objFile) {
        
        BufferedReader br = null;
        
        try {
            
            br = new BufferedReader (new FileReader (objFile));
            
            /* total number of data */
            int nObj = Integer.parseInt (br.readLine ());
            
            for (int i = 0; i < nObj; ++i) {
                
                RObject obj = new RObject ();
                
                if (i == 0)
                    m_workSpace = obj;
                else
                    m_obj.add (obj);
                
                int nPt = Integer.parseInt (br.readLine ());

                for (int j = 0; j < nPt; ++j) {
                    StringTokenizer tok = new StringTokenizer (br.readLine (), " ");
                    
                    double x = 0.0, y = 0.0;
                    
                    if (tok.hasMoreElements ())
                        x = Double.parseDouble (tok.nextToken ());
                    
                    if (tok.hasMoreElements ())
                        y = Double.parseDouble (tok.nextToken ());
                    
                    obj.setPoint (x, y);
                }
                
                /* verification */
                obj.printObject ();
            }
            
            br.close ();
            
        } catch (Exception e) {
            System.out.println (e);
            
        }
    }
    
    public void setupStartPoint (String startFile) {
        
        double x = 0.0, y = 0.0;
        BufferedReader br = null;
        
        try {
            /* open the file and start to read file */
            br = new BufferedReader (new FileReader (startFile));

            /* read and parsing start */
            StringTokenizer tok = new StringTokenizer (br.readLine (), " ");

            if (tok.hasMoreElements ())
                x = Double.parseDouble (tok.nextToken ());
                    
            if (tok.hasMoreElements ())
                y = Double.parseDouble (tok.nextToken ());
                    
            m_start = new Point2D.Double (x, y);
            
            /* read and parsing end */
            tok = new StringTokenizer (br.readLine (), " ");
            
            if (tok.hasMoreElements ())
                x = Double.parseDouble (tok.nextToken ());
                    
            if (tok.hasMoreElements ())
                y = Double.parseDouble (tok.nextToken ());
            
            m_end = new Point2D.Double (x, y);
            
            br.close ();
            
        } catch (Exception e) {
            System.out.println (e);
        }
    }
    
    /* use the http://geomalgorithms.com/a13-_intersect-4.html */
    /*  
        Copyright 2001 softSurfer, 2012 Dan Sunday
        This code may be freely used and modified for any purpose
        providing that this copyright notice is included with it.
        SoftSurfer makes no warranty for this code, and cannot be held
        liable for any real or imagined damage resulting from its use.
        Users of this code must verify correctness for their application.
    */
    boolean isLinePassedThroughPolygon (Point2D a, Point2D b, RObject plgn) {
        
        double tE = 0;              // the maximum entering segment parameter
        double tL = 1;              // the minimum leaving segment parameter
        double t, N, D;             // intersect parameter t = N / D
        RVector dS = new RVector (a, b);          // the  segment direction vector
        RVector e;                   // edge vector
        
        if (a.equals (b)) {
            System.out.println ("[Error] 2 points coincide: " + a + ":" + b);
            return false;
        }
        
        for (int i = 0; i < plgn.numPoints (); i++) {
            
            int next = (i + 1) % plgn.numPoints ();
            Point2D cVertex = plgn.getPointIdx (i);
            Point2D nVertex = plgn.getPointIdx (next); 
            
            e = new RVector (cVertex, nVertex);
            
            N = e.perp (new RVector (a, cVertex)); // = -dot(ne, S.P0 - V[i])
            D = (-1) * e.perp (dS);       // = dot(ne, dS)
            
            /* assuming that abs is very small, they are parallel */
            if (Math.abs (D) < 0.000001) {
                if (N < 0)              // P0 is outside this edge, so
                    return false;       // S is outside the polygon
                else                    // S cannot cross this edge, so
                    continue;           // ignore this edge
            }

            t = N / D;
        
            if (D < 0) {            // segment S is entering across this edge
                if (t > tE) {       // new max tE
                    tE = t;
                 
                    if (tE > tL)   // S enters after leaving polygon
                        return false;
                }
            } else {                  // segment S is leaving across this edge
                if (t < tL) {       // new min tL
                    tL = t;
                    
                    if (tL < tE)   // S leaves before entering polygon
                        return false;
                }
            }
        }

        return true;
    }
    
    
    /* check if the 2 points (a, b) can connect without going through the inside of any box */
    /* traverse through everything */
    boolean isDirectlyConnected (Point2D a, Point2D b) {
        
        /* check the workspace first */
        /* if the line intersects the work space, it is not directly connected */
        if (isLinePassedThroughPolygon (a, b, m_workSpace))
            return false;
        
        /* check all the other obstacles */
        Iterator<RObject> it = m_obj.iterator ();

        while (it.hasNext ()) {
            RObject obj = it.next ();
            
            if (isLinePassedThroughPolygon (a, b, obj))
                return false;
        }
        
        return true;
    }
    
    void expandMargin () {
        /* For all polygons,        */
        /* For each node,           */
        /* 1.   calculate K and B   */
        /* 2.   derive delta-B      */
        /* 2.5  save the equation   */
        /* 3.   calculate new (x,y) */
        
        Iterator<RObject> it = m_obj.iterator ();
        
        while (it.hasNext ()) {
            RObject obj = it.next ();
            obj.expandMargin ();
        }
    }
    
    void createPaths () {
        
        /* collect all points */
        Vector<Point2D> allPoints = new Vector<Point2D> ();
        
        Iterator<RObject> it = m_obj.iterator ();
        
        while (it.hasNext ()) {
            RObject obj = it.next ();
            allPoints.addAll (obj.getExpandedVertex ());
        }
        
        /* pair the points */
    }
    
    /* Dijstra's algorithm */
    void selectPath () {
        
    }
    
    void output () {
    }
    
    public void planPath () {
        
        expandMargin ();
        createPaths ();
        selectPath ();
        output ();
    }
    
    public static void main (String args[]) {

        if (args.length != 2) {
            System.out.println ("Error: Two params are mandatory.");
            return;
        }
        
        System.out.println ("input file: " + args[0] + ":" + args[1]);
        
        PathPlanner pp = new PathPlanner ();
        pp.createObjects (args[0]);
        pp.setupStartPoint (args[0]);
        pp.planPath ();
    }
}