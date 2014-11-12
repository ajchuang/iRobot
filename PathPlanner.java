import java.util.ArrayList;
import java.awt.geom.Point2D;
import java.io.BufferedReader;
import java.io.FileReader;
import java.util.StringTokenizer;

public class PathPlanner {
    
    /* constants */
    final double m_robotDiameter    = 35.0;
    final double m_robotRadius      = 35.0 / 2;
    
    /* data points */
    RObject m_workSpace;
    ArrayList<RObject> m_obj;
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
    
    /* check if the 2 points (a, b) can connect without going through the inside of any box */
    boolean isDirectlyConnected (Point2D a, Point2D b) {
        return true;
    }
    
    void expandMargin () {
    }
    
    void createPaths () {
    }
    
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