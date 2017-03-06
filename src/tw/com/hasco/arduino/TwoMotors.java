/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package tw.com.hasco.arduino;

import java.util.logging.Level;
import java.util.logging.Logger;
import jssc.SerialPortException;
import tw.com.hasco.MSFS.Debugger;

/**
 *
 * @author DELL calculate the needed pulse width for 6 motors from thetas of the
 * motors
 */
public class TwoMotors {

    ControlArduino controArduino;
    int servo_pos[];
    double servo_mult = 2000.0 / Math.PI;
    int dead_zone = 10;
    int servo_zero[] = {1500, 1500};
    int servo_max = 2055, servo_min = 944; // max and min rotation angle

    /**
     * This constructor try to connected com port, if fail throw exception to
     * notify the caller to treat
     *
     * @throws SerialPortException
     */
    public TwoMotors() throws SerialPortException {
        controArduino = new ControlArduino("com5");
        servo_pos = new int[2];
        servo_pos[0] = 1500;
        servo_pos[1] = 1500;
        double thetas[] = new double[2];
        updateMotors(thetas);
    }

    /**
     *
     * @param thetas : thetas in radions to control motors
     */
    public final void updateMotors(double thetas[]) {
        String str = "";
        for (int i = 0; i < 2; ++i) {
            int temp_pos = servo_zero[i] + (int) (thetas[i] * servo_mult);
            temp_pos = (temp_pos > servo_max) ? servo_max : ((temp_pos < servo_min) ? servo_min : temp_pos);
            servo_pos[i] = (Math.abs(temp_pos - servo_pos[i]) > dead_zone) ? temp_pos : servo_pos[i];
            str += String.format("%d,", servo_pos[i]);           
        }
        str = str.substring(0, str.length()-1);
        // Debugger.log(str);
        try {
            controArduino.updateMachine(str);
        } catch (SerialPortException ex) {
            Logger.getLogger(TwoMotors.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
    public void main(String args[]) {
        System.out.println("");
    }
}
