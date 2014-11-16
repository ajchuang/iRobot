import java.util.Iterator;
import java.util.ArrayList;
import java.util.Vector;
import java.util.StringTokenizer;
import java.io.BufferedReader;
import java.io.FileReader;
import java.awt.geom.Point2D;

public class PathPlanner {
    
    /* constants */
    final double m_robotDiameter    = 35.0;
    final double m_robotRadius      = 35.0 / 2;
    final double m_inf              = 9999999999.99;
    
    /* data points */
    RObject m_workSpace;
    ArrayList<RObject> m_obj;
    Vector<Point2D> m_path;

    /* 2 special points: start & end */
    Point2D.Double m_start;
    Point2D.Double m_end;
    
    public PathPlanner () {
        m_obj = new ArrayList<RObject> ();
        m_path = new Vector<Point2D> ();
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
    
    double distance (Point2D src, Point2D dst) {
        return Math.sqrt (
                    (dst.getX () - src.getX ()) * (dst.getX () - src.getX ()) + 
                    (dst.getY () - src.getY ()) * (dst.getY () - src.getY ()));  
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
        
        Iterator<RObject> it = m_obj.iterator ();
        
        while (it.hasNext ()) {
            RObject obj = it.next ();
            obj.expandMargin ();
        }
    }
    
    void createAndSelectPath () {
        
        /* collect all points */
        Vector<Point2D> allPoints = new Vector<Point2D> ();
        
        /* init the all points array */
        allPoints.add (m_start);
        
        Iterator<RObject> it = m_obj.iterator ();
        
        while (it.hasNext ()) {
            RObject obj = it.next ();
            allPoints.addAll (obj.getExpandedVertex ());
        }
        
        allPoints.add (m_end);
        
        /* pair the points */
        int s = allPoints.size ();
        double map[][] = new double[s][s];
        
        /* reset the point map table */
        for (int i = 0; i < s; ++i) {
            for (int j = 0; j < s; ++j) {
                if (i == j)
                    map[i][j] = 0.0;
                else
                    map[i][j] = -1.0;
            }
        }
                
        /* fill the point map table */
        for (int i = 0; i < s; ++i) {
            for (int j = i + 1; j < s; ++j) {
                
                Point2D src = allPoints.get (i);
                Point2D dst = allPoints.get (j);
                
                if (isDirectlyConnected (src, dst)) {
                    map[i][j] = distance (src, dst);
                    map[j][i] = map[i][j];
                }
            }
        }
        
        /* do Dijstra's algorithm */
        selectPath (map, s, allPoints);
    }
    
    /* Run Dijstra's algorithm */
    void selectPath (double[][] edges, int dim, Vector<Point2D> points) {
        
        /* selected vertex */
        Vector<Point2D> sv = new Vector<Point2D> ();
        
        /* running the dijstra's algo */
        while (points.size () > 0) {
        }
        
        
        /*
        while (true) {
            
            for (int i = 0; i < idx.size(); ++i) {
                
                int cIdx = idx.get(i);
                
                for (int j = 0; j < dim; ++j) {
                    
                    if (map[cIdx][j] != -1 && map[cIdx][j] +  < tbl[i][j]) {
                        tbl[cIdx][j] = tbl[cIdx][
                    }
                }
            }
        }
        */
    }
    
    void output () {
    }
    
    public void planPath () {
        
        expandMargin ();
        createAndSelectPath ();
        output ();
    }
    
    public static void main (String args[]) {

        if (args.length != 2) {
            System.out.println ("Error: Two params are mandatory.");
            return;
        }
        
        System.out.println ("object config file: "      + args[0]);
        System.out.println ("start-end config file: "   + args[1]);
        
        PathPlanner pp = new PathPlanner ();
        pp.createObjects (args[0]);
        pp.setupStartPoint (args[0]);
        pp.planPath ();
    }
}