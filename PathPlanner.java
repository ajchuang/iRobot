import java.util.Iterator;
import java.util.ArrayList;
import java.util.Vector;
import java.util.StringTokenizer;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.PrintWriter;
import java.awt.geom.Point2D;

/* UI components */
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.BasicStroke;
import java.awt.geom.Line2D;

import javax.swing.JOptionPane;
import javax.swing.JComponent;
import javax.swing.SwingUtilities;

public class PathPlanner extends JComponent {
    
    /* constants */
    final double m_robotDiameter    = 0.35;         /* in meters */
    final double m_robotRadius      = 0.35 / 2.0;   /* in meters */
    final double m_inf              = 9999999999.99;
    
    /* data points */
    RObject m_workSpace;
    ArrayList<RObject> m_obj;
    Vector<Point2D> m_path;
    double[][] m_map;
    Vector<Point2D> m_allpoints;

    /* 2 special points: start & end */
    Point2D.Double m_start;
    Point2D.Double m_end;
    
    /* assuming that the world coordinate is 17x17 (in meters) */
    public static Point2D transformCoord (Point2D p) {
        Point2D nP = new Point2D.Double ((12 - p.getY ()) * 50, (8.5 - p.getX ()) * 50);
        return nP;
    }
    
    void log (String s) {
        System.out.println (s);
    }
    
    public PathPlanner () {
        
        super();
        setPreferredSize (new Dimension(1100, 850));
        
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
        
        System.out.println ("Start  : " + m_start);
        System.out.println ("End    : " + m_end);
    }
    
    double distance (Point2D src, Point2D dst) {
        double dist = Math.sqrt (
                    (dst.getX () - src.getX ()) * (dst.getX () - src.getX ()) + 
                    (dst.getY () - src.getY ()) * (dst.getY () - src.getY ()));
        return dist;
    }
    
    // Ref: http://geomalgorithms.com/a05-_intersect-1.html#intersect2D_2Segments()
    // calculate if (a,b) and (c,d) have intersection
    // return (m_inf, m_inf) = completely the same line
    // return null = no solution
    // return point = valid solution
    Point2D calculateIntersection (Point2D s1_p0, Point2D s1_p1, Point2D s2_p0, Point2D s2_p1) {

        /* they are the same line */
        if ((s1_p0.equals (s2_p0) && s1_p1.equals (s2_p1)) || 
            (s1_p0.equals (s2_p1) && s1_p1.equals (s2_p0)))
            return new Point2D.Double (m_inf, m_inf);
            
        /* both lines are connecting at the end - not connected */
        if (s1_p0.equals (s2_p0) || s1_p0.equals (s2_p1) ||
            s1_p1.equals (s2_p0) || s1_p1.equals (s2_p1))
            return null;
            
        RVector u = new RVector (s1_p0, s1_p1);
        RVector v = new RVector (s2_p0, s2_p1);
        RVector w = new RVector (s2_p0, s1_p0);
        double D = RVector.perp (u, v);
        
        /* Paralell case - we highly simplified, maybe buggy - some path will be skipped */
        if (Math.abs(D) < 0.01) {
            if (RVector.perp(u,w) != 0 || RVector.perp(v,w) != 0)  {
                return null;
            }
        }
        
        double sI = RVector.perp (v,w) / D;
        double tI = RVector.perp (u,w) / D;
        
        if ((sI < 0 || sI > 1) || (tI < 0 || tI > 1)) 
            return null;

        Point2D p = new Point2D.Double ();
        p.setLocation (s1_p0.getX() + sI * u.getVecX(), s1_p0.getY() + sI * u.getVecY());
        return p;        
    }
    
    // NOT USING CN-COUNTING. Use rectangle assumption
    boolean isPointInPolygon (Point2D p, RObject plgn) {
        
        int cn = 0;
        int npt = plgn.numPoints();
        
        if (true) {
            
            double max_x = -100.0, max_y = -100.0, min_x = 100.0, min_y = 100.0;
            
            for (int i = 0; i < npt; ++i) {
                Point2D v = plgn.getExpPointIdx (i);
                double cX = v.getX ();
                double cY = v.getY ();
                
                if (cX > max_x) {
                    max_x = cX;
                }
                
                if (cX < min_x) {
                    min_x = cX;
                }
                
                if (cY > max_y) {
                    max_y = cY;
                }
                
                if (cY < min_y) {
                    min_y = cY;
                }
            }
            
            /* assume to be rectangle */
            if ((p.getX () < max_x && p.getX () > min_x) &&
                (p.getY () < max_y && p.getY () > min_y)) {               
                return true;
            } else {
                return false;
            }
            
        } else {        
            for (int i = 0; i < npt; ++i) {
                
                int next = (i + 1) % npt;
                Point2D v_i     = plgn.getExpPointIdx (i);
                Point2D v_i_1   = plgn.getExpPointIdx (next);
                
                if (((v_i.getY () <  p.getY ()) && (v_i_1.getY () >  p.getY ())) || 
                    ((v_i.getY () >  p.getY ()) && (v_i_1.getY () <  p.getY ()))) {
                
                    // compute  the actual edge-ray intersect x-coordinate
                    double vt = (double)(p.getY () - v_i.getY ()) / (v_i_1.getY () - v_i.getY ());
                    
                    if (p.getX () < (v_i.getX () + vt * (v_i_1.getX () - v_i.getX ())))
                        ++cn;
                }
            }
            
            if (cn % 2 == 0)
                return false;
            else {
                log ("Point " + p + " is in Polygon " + plgn);
                return true;
            }
        }
    }
        
    /* this is v2 - just simply test if a line has been passed a certain boundary.*/
    boolean isLinePassedThroughPolygon (Point2D a, Point2D b, RObject plgn) {
        
        int npt = plgn.numPoints();
        
        // check if the points are in the inside of the polygon
        for (int i = 0; i < npt; i++) {
            if (plgn != m_workSpace) {
                if (isPointInPolygon (a, plgn) == true || 
                    isPointInPolygon (b, plgn) == true)
                    return true;
            }
        }
        
        for (int i = 0; i < npt; i++) {
            int next = (i + 1) % npt;
            Point2D v_i = plgn.getPointIdx (i);
            Point2D v_i_1 = plgn.getPointIdx (next);
            
            Point2D intr = calculateIntersection (a, b, v_i, v_i_1);
            
            if (intr == null)
                continue;
            else if (intr.getX () == m_inf && intr.getY () == m_inf)
                return false;
            else {
                log ("Intersection between ( " + a + ":" + b + " ) and ( " + v_i + ":" + v_i_1 + " ) is " + intr);
                return true;
            }
        }
        
        for (int i = 0; i < npt; i++) {
            int next = (i + 1) % npt;
            Point2D v_i = plgn.getExpPointIdx (i);
            Point2D v_i_1 = plgn.getExpPointIdx (next);
            
            Point2D intr = calculateIntersection (a, b, v_i, v_i_1);
            
            if (intr == null)
                continue;
            else if (intr.getX () == m_inf && intr.getY () == m_inf)
                return false;
            else {
                log ("Intersection between ( " + a + ":" + b + " ) and ( " + v_i + ":" + v_i_1 + " ) is " + intr);
                return true;
            }
        }
        
        /* hit nothing */
        return false;
    }
    
    /* check if the 2 points (a, b) can connect without going through the inside of any box */
    /* traverse through everything */
    boolean isDirectlyConnected (Point2D a, Point2D b) {
        
        /* check the workspace first */
        /* if the line intersects the work space, it is not directly connected */
        if (isLinePassedThroughPolygon (a, b, m_workSpace)) {
            System.out.println ("Hit the boundary: " + a + ":" + b);
            return false;
        }
        
        /* check all the other obstacles */
        for (int i=0; i<m_obj.size(); ++i) {
            
            RObject obj = m_obj.get (i);
            
            if (isLinePassedThroughPolygon (a, b, obj)) {
                System.out.println ("Line " + a + ":" + b + " hits obj " + i);
                return false;
            }
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
        m_allpoints = new Vector<Point2D> ();
        
        /* init the all points array */
        m_allpoints.add (m_start);
        
        Iterator<RObject> it = m_obj.iterator ();
        
        while (it.hasNext ()) {
            RObject obj = it.next ();
            m_allpoints.addAll (obj.getExpandedVertex ());
        }
        
        m_allpoints.add (m_end);
        
        /* pair the points */
        int s = m_allpoints.size ();
        m_map = new double[s][s];
        
        /* reset the point map table */
        for (int i = 0; i < s; ++i) {
            for (int j = 0; j < s; ++j) {
                if (i == j)
                    m_map[i][j] = 0.0;
                else
                    m_map[i][j] = -1.0;
            }
        }
                
        /* fill the point map table */
        for (int i = 0; i < s; ++i) {
            for (int j = 0; j < s; ++j) {
                
                if (i == j)
                    continue;
                
                Point2D src = m_allpoints.get (i);
                Point2D dst = m_allpoints.get (j);
                
                if (isDirectlyConnected (src, dst)) {
                    System.out.println (src + ":" + dst + " is connected");
                    m_map[i][j] = distance (src, dst);
                } else {
                    m_map[i][j] = -1.0;
                }
            }
        }
        
        /* do Dijstra's algorithm */
        selectPath (m_map, m_allpoints);
    }
    
    /* Run Dijstra's algorithm */
    /* '0' has to be the starting point, dim-1 has to be the end point */
    void selectPath (double[][] edges, Vector<Point2D> allpoints) {
        
        /* local variables */
        int dim = allpoints.size ();
        Vector<Point2D> s = new Vector<Point2D> ();
        Vector<Point2D> q = new Vector<Point2D> (allpoints);
        double[] d = new double[dim];
        int[] prev = new int[dim];
        
        /* init d and prev */
        for (int i = 0; i < dim; ++i) {
            d[i] = m_inf;
            prev[i] = -1;
        }
        
        d[0] = 0.0;
        
        /* running the dijstra's algo */
        while (q.size () > 0) {
            
            Point2D u = null;
            double min = m_inf;
            int minIdx = -1;
            
            /* extract min */
            for (int i = 0; i < q.size (); ++i) {
                if (d[i] < min) {
                    minIdx = i;
                    min = d[i];
                    u = q.get (i);
                    q.remove (u);
                }
            }
            
            int u_idx = allpoints.indexOf (u);
            
            if (u != null && min < m_inf) {
                System.out.println ("Pt " + u_idx + "\t ( " + u + " ) is selected.");
                s.add (u);
            } else {
                System.out.println ("No more min ?");
                return;
            }
            
            /* update the d table */
            for (int v_idx = 0; v_idx < dim; ++v_idx) {
                
                double len = edges[u_idx][v_idx];
                
                if (len == -1) {
                    /* not directly connected */
                    continue;
                }
                
                if (d[v_idx] > d[u_idx] + len) {
                    
                    double old_d = d[v_idx];
                    d[v_idx] = d[u_idx] + len;
                    prev[v_idx] = u_idx;
                    
                    System.out.println ("Update: " + v_idx + " from " + u_idx + " cost: " + old_d + " --> " + d[v_idx]);
                }
            }
            
            /* Press any key to continue */
            //System.in.read ();
        }
        
        int x = dim - 1;
        
        for (int i = 0; i < dim; ++i)
            System.out.println ("Node " + i + " --> prev: " + prev[i]);
            
        int nxtPoint = prev[dim - 1];
        int cnt = 0;
        Vector<Integer> revPath = new Vector<Integer> ();
        revPath.add (Integer.valueOf (dim - 1));
        revPath.add (Integer.valueOf (nxtPoint));
        log ("Prev step = " + nxtPoint);
        
        while (nxtPoint != -1) {
            nxtPoint = prev[nxtPoint];
            
            if (nxtPoint == -1)
                break;
            
            revPath.add (Integer.valueOf (nxtPoint));
            log ("Prev step = " + nxtPoint);
            
            cnt++;
            
            if (cnt > dim) {
                log ("no valid path");
                return;
            }
        }
        
        try {
            PrintWriter writer = new PrintWriter("route.txt", "US-ASCII");
            
            // output the actual path
            for (int i = revPath.size() - 1; i >= 0; --i) {
                Point2D p = allpoints.get(revPath.get(i).intValue());
                m_path.add (p);
                log ("Path: " + p);
                
                writer.println (p.getX () + " " + p.getY ());
            }
            
            writer.close();
        } catch (Exception e) {
            log ("Oooops: " + e);
        }
    }
    
    public void planPath () {
        expandMargin ();
        createAndSelectPath ();
        System.out.println ("*** completed ***");
    }
    
    
    public void paintComponent(Graphics g) {
        
        g.setColor(Color.white);
        g.fillRect(0, 0, getWidth(), getHeight());
        Dimension d = getPreferredSize();
        
        /* draw the start */
        Point2D new_start = transformCoord (m_start);
        g.setColor (Color.yellow);
        g.fillOval (
            (int)(new_start.getX () - 17.5 * 0.5),
            (int)(new_start.getY () - 17.5 * 0.5),
            (int)(35.0 * 0.5), 
            (int)(35.0 * 0.5));
        
        /* draw the end */
        Point2D new_end = transformCoord (m_end);
        g.setColor (Color.red);
        g.fillOval (
            (int)(new_end.getX () - 17.5 * 0.5),
            (int)(new_end.getY () - 17.5 * 0.5),
            (int)(35.0 * 0.5),
            (int)(35.0 * 0.5));
        
        /* draw the workspace */
        Graphics2D g22 = (Graphics2D) g;
        g22.setStroke (new BasicStroke (5));
        m_workSpace.paint (g22, Color.gray);
        
        g22.setStroke (new BasicStroke (1));
        /* draw the objects */
        for (int i = 0; i < m_obj.size (); ++i)
            m_obj.get(i).paint (g22, Color.cyan);
            
        /* draw the m_map */
        for (int i = 0; i < m_allpoints.size (); ++i) {
            for (int j = 0; j < m_allpoints.size (); ++j) {
                if (m_map[i][j] != -1.0) {
                    Point2D src = transformCoord (m_allpoints.get (i));
                    Point2D dst = transformCoord (m_allpoints.get (j));
                    
                    g22.setColor (Color.black);
                    g22.drawLine (
                        (int) src.getX (),
                        (int) src.getY (),
                        (int) dst.getX (),
                        (int) dst.getY ());
                }
            }
        }
        
        /* draw the final path */
        for (int i = 0; i < m_path.size () - 1; ++i) {
            Point2D src = transformCoord (m_path.get (i));
            Point2D dst = transformCoord (m_path.get (i + 1));
                    
            g22.setColor (Color.red);
            g22.setStroke (new BasicStroke (5));
            g22.drawLine (
                (int) src.getX (),
                (int) src.getY (),
                (int) dst.getX (),
                (int) dst.getY ());
        }
    }
    
    public static void main (String args[]) {

        if (args.length != 2) {
            System.out.println ("Error: Two params are mandatory.");
            return;
        }
        
        System.out.println ("object config file: "      + args[0]);
        System.out.println ("start-end config file: "   + args[1]);
        
        final String n0 = new String (args[0]);
        final String n1 = new String (args[1]);
        
        Runnable r = new Runnable() {
            public void run() {
                PathPlanner pp = new PathPlanner ();
                pp.createObjects (n0);
                pp.setupStartPoint (n1);
                pp.planPath ();
                
                JOptionPane.showMessageDialog(null, pp);
            }
        };
        
        SwingUtilities.invokeLater(r);
        
        
    }
}