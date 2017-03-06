/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package tw.com.hasco.arduino;

import static java.lang.Thread.sleep;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.logging.Level;
import java.util.logging.Logger;
import jssc.SerialPortException;
import tw.com.hasco.MSFS.Debugger;
import tw.com.hasco.MSFS.FS.FSBasic;
import tw.com.hasco.MSFS.Observer;

/**
 *
 * @author DELL
 */
public class StewPlatform implements Observer, Runnable {

    /*
     * ******************************************************************************
     * P1 = translation in X direction (sway) 
     * S* = Sin(theta_*) 
     * P2 = translation in Y direction (surge) 
     * C* = Cos(theta_*) 
     * P3 = translation in Z direction (heave) e.g. C1 = Cos(theta_1)
     *
     * General Matrix for Yaw-Pitch-Roll-Sway-Surge-Heave Transformation 
     * | C2C3  S1S2C3 - C1S3  C1S2C3 + S1S3  P1 |   | X |   | q[0] | 
     * | C2S3  S1S2S3 + C1C3  C1S2S3 - S1C3  P2 | x | Y | = | q[1] | 
     * | -S2   S1C2           C1C2           P3 |   | Z |   | q[2] |
     * | 0     0              0              1  |   | 1 |   | 1    | 
     * where: X = L1 theta_1 = 0 Y = 0 theta_2 = theta_a
     * Z = 0 theta_3 = theta_s
     * ******************************************************************************
     */
    /*
     * theta_r = angle between attachment points 
     * theta_p = angle between rotation points 
     * theta_s = orientation of the servos 
     * RD = distance to end effector attachment points 
     * PD = distance to servo rotation points 
     * L1 = servo arm length 
     * L2 = connecting arm length 
     * z_home = default z height with servo arms horizontal 
     * servo_min = lower limit for servo arm angle
     * servo_max = upper limit for servo arm angle 
     * servo_mult = multiplier to convert to milliseconds 
     * re = location of attachment points in end effector frame [x/y][1-6] 
     * pe = location and orientation of end effector frame relative to the base frame [sway, surge, heave, pitch, roll, yaw)
     * theta_a = angle of the servo arm 
     * servo_pos = value written to each servo
     * q = position of lower mounting point of connecting link [x,y,x][1-6] 
     * r = position of upper mounting point of connecting link 
     * dl = difference between x,y,z coordinates of q and r 
     * dl2 = distance between q and r
     */
    boolean running = true;
    final boolean debug = true;
    final double PI = Math.PI;
    final double THETA[] = {PI / 6.0, PI / 6.0, -PI / 2.0, -PI / 2.0, -7.0 / 6.0 * PI, -7.0 / 6.0 * PI};
    double theta_r = Math.toRadians(20.0),
            theta_p = Math.toRadians(20.0),
            theta_s[] = {2 * PI / 3, -PI / 3, 0, PI, -2 * PI / 3, PI / 3},
            RD = 60,
            PD = 60,
            L1 = 30,
            L2 = 210,
            z_home = 207,
            theta_min = -50.0 * PI / 180.0, // theta min
            theta_max = 50.0 * PI / 180.0; // theta_max
    double p[][], re[][];
    double pe[], theta_a[], theta_a1[], servo_pos[], q[][], r[][], dl[][], dl2[], motor_theta[];
    // Observer
    private LinkedList<SPObserver> observers;
    Motors motors;

    /**
     * this for after initilized, calculate the relative 0 position.
     */
    private void calPRE() {
        for (int i = 0; i < 3; ++i) {
            p[0][2 * i] = PD * Math.cos(THETA[2 * i] + theta_p);
            p[1][2 * i] = PD * Math.sin(THETA[2 * i] + theta_p);
            p[2][2 * i] = 0.0;
            p[0][2 * i + 1] = PD * Math.cos(THETA[2 * i + 1] - theta_p);
            p[1][2 * i + 1] = PD * Math.sin(THETA[2 * i + 1] - theta_p);
            p[2][2 * i + 1] = 0.0;
        }
        for (int i = 0; i < 3; ++i) {
            re[0][2 * i] = RD * Math.cos(THETA[2 * i] + theta_r);
            re[1][2 * i] = RD * Math.sin(THETA[2 * i] + theta_r);
            re[2][2 * i] = 0.0;
            re[0][2 * i + 1] = RD * Math.cos(THETA[2 * i + 1] - theta_r);
            re[1][2 * i + 1] = RD * Math.sin(THETA[2 * i + 1] - theta_r);
            re[2][2 * i + 1] = 0.0;
        }
    }

    /**
     * use to calculate the motor arm angles, after given pe(x, y,z, pitch,
     * roll, yaw), using recusive calculation to get correct angles
     */
    private void recCalAngle() {
        for (int i = 0; i < 6; i++) {
            q[0][i] = L1 * Math.cos(theta_a1[i]) * Math.cos(theta_s[i]) + p[0][i];
            q[1][i] = L1 * Math.cos(theta_a1[i]) * Math.sin(theta_s[i]) + p[1][i];
            q[2][i] = L1 * Math.sin(theta_a1[i]);
            r[0][i] = re[0][i] * Math.cos(pe[4]) * Math.cos(pe[5])
                    + re[1][i] * (Math.sin(pe[3]) * Math.sin(pe[4]) * Math.cos(pe[5]) - Math.cos(pe[3]) * Math.sin(pe[5]))
                    + pe[0];
            r[1][i] = re[0][i] * Math.cos(pe[4]) * Math.sin(pe[5])
                    + re[1][i] * (Math.cos(pe[3]) * Math.cos(pe[5]) + Math.sin(pe[3]) * Math.sin(pe[4]) * Math.sin(pe[5]))
                    + pe[1];
            r[2][i] = -re[0][i] * Math.sin(pe[4]) + re[1][i] * Math.sin(pe[3]) * Math.cos(pe[4])
                    + z_home + pe[2];
            dl[0][i] = q[0][i] - r[0][i];
            dl[1][i] = q[1][i] - r[1][i];
            dl[2][i] = q[2][i] - r[2][i];
            dl2[i] = Math.sqrt(dl[0][i] * dl[0][i] + dl[1][i] * dl[1][i] + dl[2][i] * dl[2][i])
                    - L2;
            theta_a1[i] += dl2[i] / L2;
        }
    }

    /**
     *
     */
    private void calRotAngle() {
        for (int i = 0; i < 50; ++i) {
            recCalAngle();
        }
        String str = "";
        for (int i = 0; i < 6; i++) {
            theta_a1[i] += dl2[i] / L2;
            theta_a1[i] = (theta_a1[i] < theta_min) ? theta_min
                    : ((theta_a1[i] > theta_max) ? theta_max : theta_a1[i]);
            str += (int) (180.0 / Math.PI * theta_a1[i]) + ",";
        }
        synchronized (this) {
            System.arraycopy(theta_a1, 0, theta_a, 0, theta_a1.length);
        }
        Debugger.log(str);
        notifyObservers();
    }

    public double[][] getP() {
        return p;
    }

    public double[][] getQ() {
        return q;
    }

    public double[][] getR() {
        return r;
    }

    /**
     * add observer
     *
     * @param o : oberver of this
     */
    public void addOberver(SPObserver o) {
        observers.add(o);

    }

    /**
     * use to notify the related observer
     */
    private void notifyObservers() {
        observers.forEach((m) -> {
            m.update(this);
        });
    }

    public StewPlatform() throws SerialPortException {
        observers = new LinkedList<>();
        theta_r = Math.toRadians(20); // set theta r
        theta_p = Math.toRadians(20); // set theta p
        theta_s = new double[6];
        double theta[] = {2 * PI / 3, -PI / 3, 0, PI, -2 * PI / 3, PI / 3};
        for (int i = 0; i < 6; ++i) {
            theta_s[i] = theta[i];
        }
        RD = 60.0;
        PD = 60.0;
        L1 = 30.0;
        L2 = 210;
        z_home = 207;
        p = new double[3][6];
        re = new double[3][6];
        calPRE();
        pe = new double[6];
        theta_a = new double[6];
        theta_a1 = new double[6];
        motor_theta = new double[6];
        servo_pos = new double[6];
        q = new double[3][6];
        r = new double[3][6];
        dl = new double[3][6];
        dl2 = new double[6];
        try {
            motors = new Motors();
        } catch (SerialPortException ex) {
            motors = null;
            Logger.getLogger(StewPlatform.class.getName()).log(Level.SEVERE, null, ex);
        }
        calRotAngle();
    }

    public void setInitParams(ArrayList<Double> ps) {
        int i = 0;
        theta_r = Math.toRadians(ps.get(i++));
        theta_p = Math.toRadians(ps.get(i++));
        theta_s[0] = Math.toRadians(ps.get(i++));
        theta_s[1] = Math.toRadians(ps.get(i++));
        theta_s[2] = Math.toRadians(ps.get(i++));
        theta_s[3] = Math.toRadians(ps.get(i++));
        theta_s[4] = Math.toRadians(ps.get(i++));
        theta_s[5] = Math.toRadians(ps.get(i++));
        RD = Math.toRadians(ps.get(i++));
        PD = Math.toRadians(ps.get(i++));
        L1 = Math.toRadians(ps.get(i++));
        L2 = Math.toRadians(ps.get(i++));
        z_home = Math.toRadians(ps.get(i++));
        // servo_min = Math.toRadians(ps.get(i++));
        // servo_max = Math.toRadians(ps.get(i++));
        // feedback = 1.0 / Math.toRadians(ps.get(i++));
        // servo_mult = Math.toRadians(ps.get(i++)) / PI;
        calPRE();
        calRotAngle();

    }

    public void setX(double d) {
        pe[0] = d;
        calRotAngle();
    }

    public void setY(double d) {
        pe[1] = d;
        calRotAngle();
    }

    public void setZ(double d) {
        pe[2] = d;
        calRotAngle();
    }

    public void setPitch(double d) {
        pe[3] = d;
        calRotAngle();
    }

    public void setRoll(double d) {
        pe[4] = d;
        calRotAngle();
    }

    public void setYaw(double d) {
        pe[5] = d;
        calRotAngle();
    }

    /**
     * The problem is used to receive setting of motion
     *
     * @param params : z, y, z, picth, roll, yaw
     */
    public void setParams(double[] params) {
        for (int i = 0; i < 6; ++i) {
            pe[i] = params[i];
        }
        calRotAngle();
    }

    public void updateMotors() {
        if (motors == null) {
            return;
        }
        synchronized (this) {
            for (int i = 0; i < 6; ++i) {
                if (i % 2 == 1) {
                    motor_theta[i] = theta_a[i];
                } else {
                    double theta = -theta_a[i];
                    motor_theta[i] = -theta_a[i];
                }
            }
        }
        motors.updateMotors(motor_theta);
        try {
            sleep(100);
        } catch (InterruptedException ex) {
            Logger.getLogger(StewPlatform.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        try {
            // TODO code application logic here
            StewPlatform sp = new StewPlatform();
            sp.setZ(3);
            sp.setZ(0);
        } catch (SerialPortException ex) {
            Logger.getLogger(StewPlatform.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    @Override
    public void update(FSBasic fsBasic) {
        pe[3] = Math.toRadians(fsBasic.pitch());
        pe[4] = Math.toRadians(fsBasic.bank());
        // don't take the yaw angle
        // pe[5] = Math.toRadians(fsBasic.heading() - 90.0);
        // Debugger.log(fsBasic.pitch() + "," + fsBasic.bank() + "," + fsBasic.heading());
        calRotAngle();
    }

    public void stop() {
        running = false;
    }

    public void start() {
        running = true;
    }

    @Override
    public void run() {
        if (motors == null) {
            return;
        }
        while (running) {
            updateMotors();
        }
    }

}
