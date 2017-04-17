/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package tw.com.hasco.arduino;

import jssc.*;
import tw.com.hasco.MSFS.Debugger;

// import tw.com.hasco.MSFS.Debugger;
/**
 * send information to arduino through com=comName port
 *
 * @author DELL
 *
 */
public class ControlArduino {

    SerialPort serialPort;

    public ControlArduino(String comName) throws SerialPortException {
        serialPort = new SerialPort(comName);
        serialPort.openPort();
        serialPort.setParams(SerialPort.BAUDRATE_9600,
                SerialPort.DATABITS_8,
                SerialPort.STOPBITS_1,
                SerialPort.PARITY_NONE);

        serialPort.setFlowControlMode(SerialPort.FLOWCONTROL_RTSCTS_IN
                | SerialPort.FLOWCONTROL_RTSCTS_OUT);
    }

    public void closePort() throws SerialPortException {
        if (serialPort != null && serialPort.isOpened()) {
            serialPort.purgePort(1);
            serialPort.purgePort(2);
            serialPort.closePort();
        }

    }

    public void updateMachine(String str) throws SerialPortException {
        serialPort.writeString(str);
        Debugger.log("input: " + str);
        String str1 = serialPort.readString();
        Debugger.log("output: " + str1);
    }

}
