/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package tw.com.hasco.arduino;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Polygon;
import java.awt.RenderingHints;
import javax.swing.JFrame;
import jssc.SerialPortException;
import tw.com.hasco.arduino.StewPlatform;

/**
 *
 * @author DELL
 */
public class Plot2DPanel extends javax.swing.JPanel {

    double sx0, sz0;
    int cxmax, cxmin;
    int czmax, czmin;
    double mag = 4.0;
    StewPlatform sp;

    /**
     * Creates new form PlotPanel
     */
    public Plot2DPanel() {
        initComponents();
        sx0 = 200;
        sz0 = 300;
        cxmax = 180;
        cxmin = -180;
        czmax = 280;
        czmin = -80;
        sp = null;
    }

    public void setSp(StewPlatform sp) {
        this.sp = sp;
    }

    public Point trasform3Dto2D(double x, double y, double z) {
        int sx = (int) (sx0 + mag * x);
        int sz = (int) (sz0 - z * mag);
        return new Point(sx, sz);
    }

    private void drawAxis(Graphics2D g2d) {
        int p11_x = (int) (sx0 + cxmin);
        int p11_y = (int) (sz0);
        int p12_x = (int) (sx0 + cxmax);
        int p12_y = (int) (sz0);

        int p31_x = (int) (sx0);
        int p31_y = (int) (sz0 - czmin);
        int p32_x = (int) (sx0);
        int p32_y = (int) (sz0 - czmax);
        g2d.setColor(Color.black);
        g2d.drawLine(p11_x, p11_y, p12_x, p12_y);
        g2d.drawLine(p31_x, p31_y, p32_x, p32_y);
    }

    private void drawSP(Graphics2D g2d) {
        if (sp == null) {
            return;
        }
        double p[][] = sp.getP(); // motor center, is equal zero
        double q[][] = sp.getQ(); // the L2 end on bottom
        double r[][] = sp.getR(); // the L2 end on top
        // trasform into plane x y z;
        Point p_p[] = new Point[6];
        Point p_q[] = new Point[6];
        Point p_r[] = new Point[6];
        for (int i = 0; i < 6; ++i) {
            p_p[i] = trasform3Dto2D(p[0][i], p[1][i], p[2][i]);
            p_q[i] = trasform3Dto2D(q[0][i], q[1][i], q[2][i]);
            p_r[i] = trasform3Dto2D(r[0][i], r[1][i], r[2][i]);
        }
        int x[] = new int[6];
        int y[] = new int[6];
        // Polygon poly;
        for (int i = 0; i < 6; ++i) {
            x[i] = p_p[i].getX();
            y[i] = p_p[i].getY();
            g2d.drawString(Integer.toString(i), x[i], y[i]);
        }
        // g2d.drawLine(x[2], y[2], x[3], y[3]);
        for (int i = 0; i < 6; ++i) {
            x[i] = p_r[i].getX();
            y[i] = p_r[i].getY();
            g2d.drawString(Integer.toString(i), x[i], y[i]);
        }
        Polygon poly = new Polygon(x, y, 6);
        g2d.setColor(new Color(0, 153, 51));
        g2d.fill(poly);
        g2d.drawLine(x[2], y[2], x[3], y[3]);
        g2d.setColor(Color.red);
        for (int i = 0; i < 6; ++i) {
            g2d.drawLine(p_p[i].getX(), p_p[i].getY(), p_q[i].getX(), p_q[i].getY());
            g2d.drawLine(p_q[i].getX(), p_q[i].getY(), p_r[i].getX(), p_r[i].getY());
        }

    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g); //To change body of generated methods, choose Tools | Templates.
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(
                RenderingHints.KEY_ANTIALIASING,
                RenderingHints.VALUE_ANTIALIAS_ON
        );
        drawAxis(g2d);
        if (sp != null) {
            drawSP(g2d);
        }
    }

    /**
     * This method is called from within the constructor to initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is always
     * regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(this);
        this.setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 400, Short.MAX_VALUE)
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 400, Short.MAX_VALUE)
        );
    }// </editor-fold>//GEN-END:initComponents
    public static void main(String[] args) throws SerialPortException {
        StewPlatform sp = new StewPlatform("com5");
        JFrame f = new JFrame("Demo");
        Plot2DPanel p = new Plot2DPanel();
        p.setSp(sp);
        f.add(p);

        f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        f.setSize(400, 400);
        f.setVisible(true);
    }

    public void update(StewPlatform sp) {
        this.sp = sp;
        repaint();
    }

    public void zoomin() {
        mag *= 2.0;
        repaint();
    }

    public void zoomout() {
        mag /= 2.0;
        repaint();
    }
    // Variables declaration - do not modify//GEN-BEGIN:variables
    // End of variables declaration//GEN-END:variables
}
