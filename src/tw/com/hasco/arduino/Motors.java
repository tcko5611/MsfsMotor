/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package tw.com.hasco.arduino;

import static java.lang.Thread.sleep;
import java.util.logging.Level;
import java.util.logging.Logger;
import jssc.SerialPortException;

/**
 * Calculate the needed pwm narrow time width for 6 motors from radians motors
 *
 * @author DELL
 */
public class Motors {

    ControlArduino controArduino;
    int servo_pos[];
    double servo_mult = 2000.0 / Math.PI;
    int dead_zone = 10;
    int servo_zero[] = {1500, 1500, 1500, 1500, 1500, 1500};
    int servo_max = 2055, servo_min = 944; // max and min rotation angle

    /**
     * This constructor try to connected com port, if fail throw exception to
     * notify the caller to treat
     *
     * @throws SerialPortException
     */
    public Motors() throws SerialPortException {
        controArduino = new ControlArduino("com5");
        servo_pos = new int[6];
        servo_pos[0] = 1500;
        servo_pos[1] = 1500;
        servo_pos[2] = 1500;
        servo_pos[3] = 1500;
        servo_pos[4] = 1500;
        servo_pos[5] = 1500;
        double thetas[] = new double[6];
        updateMotors(thetas);
    }

    /**
     *
     * @param thetas : thetas in radions to control motors
     */
    public final void updateMotors(double thetas[]) {
        String str = "";
        for (int i = 0; i < 6; ++i) {
            int temp_pos = servo_zero[i] + (int) (thetas[i] * servo_mult);
            temp_pos = (temp_pos > servo_max) ? servo_max : ((temp_pos < servo_min) ? servo_min : temp_pos);
            servo_pos[i] = (Math.abs(temp_pos - servo_pos[i]) > dead_zone) ? temp_pos : servo_pos[i];
            str += String.format("%d,", servo_pos[i]);
        }
        str = str.substring(0, str.length() - 1);
        // System.out.println(str);
        try {
            // servo_pos[5] = servo_zero[5] + (int) (thetas[5] * servo_mult);
            // str += Integer.toString(servo_pos[5]);
            controArduino.updateMachine(str);
        } catch (SerialPortException ex) {
            // just ignore it
            Logger.getLogger(Motors.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    /**
     * @param args the command line arguments
     */
    public static void main(String args[]) throws SerialPortException {

        Motors motors;

        motors = new Motors();
        for (int degree = 0; degree < 90; ++degree) {

            double theta = degree * Math.PI / 180.0;
            double thetas[] = new double[6];
            for (int i = 0; i < 6; ++i) {
                thetas[i] = theta;
            }
            motors.updateMotors(thetas);
            try {
                sleep(100);
            } catch (InterruptedException ex) {
                Logger.getLogger(Motors.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
    for (int degree = 90; degree > -90; --degree) {

            double theta = degree * Math.PI / 180.0;
            double thetas[] = new double[6];
            for (int i = 0; i < 6; ++i) {
                thetas[i] = theta;
            }
            motors.updateMotors(thetas);
            try {
                sleep(100);
            } catch (InterruptedException ex) {
                Logger.getLogger(Motors.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
    }
}
