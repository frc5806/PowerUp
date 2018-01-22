package org.usfirst.frc.team5806.robot;

import java.awt.TextArea;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.StringTokenizer;
import javax.swing.JOptionPane;

/**
 * A few useful methods.
 *
 * @author Jared
 */
public class Util {

    /**
     * Create a window asking for a number.
     *
     * @param message Prompt for the number
     * @param title Title of window
     * @return The number
     */
    public static double messageBoxDouble(String message, String title) {
        double x;
        try {
            x = Double.valueOf(JOptionPane.showInputDialog(
                    null, message, title, JOptionPane.PLAIN_MESSAGE));
        } catch (NumberFormatException e) {
            x = 0;
            StringWriter sw = new StringWriter();
            PrintWriter pw = new PrintWriter(sw);
            e.printStackTrace(pw);
            JOptionPane.showMessageDialog(null, new TextArea(sw.toString()),
                    "Number Error", 3);
        }
        return x;
    }

    /**
     * Create a window asking for a string.
     *
     * @param message prompt
     * @param title title of window
     * @return the string
     */
    public static String messageBoxString(String message, String title) {
        return JOptionPane.showInputDialog(
                null, message, title, JOptionPane.PLAIN_MESSAGE);
    }

    /**
     * Display an error message.
     *
     * @param message The message
     * @param title Title of the window.
     */
    public static void displayMessage(String message, String title) {
        JOptionPane.showMessageDialog(null, message, title, 2);
    }

    /**
     * Slope to radian. It's atan2.
     *
     * @param x
     * @param y
     * @return
     */
    public static double slopeToRadians(double x, double y) {
        return Math.atan2(y, x);
    }

    /**
     * Angle to slope. It's tangent(theta).
     *
     * @param theta
     * @return
     */
    public static double angleToSlope(double theta) {
        return Math.tan(theta);
    }

    /**
     * Create a segment group from a string of data. 
     * Based off of 254's version.
     * @param in
     * @return
     */
    public static ArrayList<ArrayList<Double>> stringToPath236(String in) {
        //a.time,  a.vel, a.acc, a.posit, a.x, a.y, a.dydx, a.d2ydx2
    	ArrayList<ArrayList<Double>> path = new ArrayList<ArrayList<Double>>();
    	ArrayList<Double> robot = new ArrayList<Double>();
    	ArrayList<Double> left = new ArrayList<Double>();
    	ArrayList<Double> right = new ArrayList<Double>();
        //string breaker.  Uses \n
        StringTokenizer tokenizer = new StringTokenizer(in, "\n");
        System.out.println("found " + tokenizer.countTokens() + " things");
        //the first line is the name.  It's compatible with team 254's paths too.
        String title = tokenizer.nextToken();

        int size = Integer.parseInt(tokenizer.nextToken());

        for (int i = 0; i < size; i++) {
            StringTokenizer lineToken = new StringTokenizer(
                    tokenizer.nextToken(), " ");
            robot.add(getNextDouble(lineToken));
            robot.add(getNextDouble(lineToken));
            double derp = 0.0;
            derp = getNextDouble(lineToken);
            derp = getNextDouble(lineToken);
            derp = getNextDouble(lineToken);
            derp = getNextDouble(lineToken);
            derp = getNextDouble(lineToken);
            derp = getNextDouble(lineToken);
        }
        
        for (int i = 0; i < size; i++) {
            StringTokenizer lineToken = new StringTokenizer(
                    tokenizer.nextToken(), " ");
            left.add(getNextDouble(lineToken));
            left.add(getNextDouble(lineToken));
            double derp = 0.0;
            derp = getNextDouble(lineToken);
            derp = getNextDouble(lineToken);
            derp = getNextDouble(lineToken);
            derp = getNextDouble(lineToken);
            derp = getNextDouble(lineToken);
            derp = getNextDouble(lineToken);
        }

        for (int i = 0; i < size; i++) {
            StringTokenizer lineToken = new StringTokenizer(
                    tokenizer.nextToken(), " ");
            right.add(getNextDouble(lineToken));
            right.add(getNextDouble(lineToken));
            double derp = 0.0;
            derp = getNextDouble(lineToken);
            derp = getNextDouble(lineToken);
            derp = getNextDouble(lineToken);
            derp = getNextDouble(lineToken);
            derp = getNextDouble(lineToken);
            derp = getNextDouble(lineToken);
        }
        
        path.add(robot);
        path.add(left);
        path.add(right);
        return path;
    }

    /**
     * Get the next double from the given tokenizer.
     * @param tok
     * @return
     */
    public static double getNextDouble(StringTokenizer tok) {
        return Double.parseDouble(tok.nextToken());
    }

    /**
     * Read a file into a string.  Not used.
     * @param path
     * @param encoding
     * @return
     * @throws IOException 
     */
    static String readFile(String path, Charset encoding)
            throws IOException {
        byte[] encoded = Files.readAllBytes(Paths.get(path));
        return new String(encoded, encoding);
    }
    
    /**
     * A better file reader. 
     * @param file
     * @return
     * @throws IOException
     */
    public static String readStringFromFile(File file) throws IOException {
        byte[] data = Files.readAllBytes(file.toPath());
        return new String(data, Charset.defaultCharset());
    }

        /**
     * Method from 2014 data viewer.
     *
     * @param x
     * @param y
     * @param name
     * @param yaxis
     */
    
    public static void main(String args[]) {
    	//stringToPath();
    }
}
