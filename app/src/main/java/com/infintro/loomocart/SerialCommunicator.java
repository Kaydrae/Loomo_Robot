package com.infintro.loomocart;

import android.hardware.usb.UsbDevice;
import android.util.Log;
import android.widget.Toast;

//libraries for communicating with arduino (https://github.com/OmarAflak/Arduino-Library)
import me.aflak.arduino.Arduino;
import me.aflak.arduino.ArduinoListener;

public class SerialCommunicator implements ArduinoListener {
    private final String TAG = "Serial Comm.";

    //arduino we are communicating with
    private Arduino mArduino;

    //constructor
    public SerialCommunicator() {
        mArduino = new Arduino(LoomoCart.getContext());     //set the arduino instance
        mArduino.addVendorId(6790);                         //add vid for elegoo nano board
    }

    //function to begin serial communications
    public void serialBegin() {
        Log.d(TAG, "Serial begin");
        mArduino.setArduinoListener(this);  //set listener for arduino
    }

    //function to end serial communications
    public void serialEnd() {
        Log.d(TAG, "Serial end");
        mArduino.unsetArduinoListener();    //unset listener
        mArduino.close();                   //close the open serial port
    }

    @Override
    public void onArduinoAttached(UsbDevice device) {
        Log.d(TAG, "Arduino attached!");
        mArduino.open(device);                  //open a serial port once arduino is attached
    }

    @Override
    public void onArduinoDetached() {
        Log.d(TAG, "Arduino detached!");
    }

    @Override
    public void onArduinoMessage(byte[] bytes) {
        Log.d(TAG, "Arduino Message Received: " + new String(bytes));
    }

    @Override
    public void onArduinoOpened() {
        Log.d(TAG, "Arduino Opened!");
        String str = "S";
        mArduino.send(str.getBytes());  //send stop signal
    }

    @Override
    public void onUsbPermissionDenied() {
        Toast.makeText(LoomoCart.getContext(), "Arduino Permission Denied, Attempting to Re-Open.", Toast.LENGTH_LONG);
        mArduino.reopen();  //re-open arduino if user denied permission
    }


    public void sendBytes(byte[] bytes) {
        if (mArduino.isOpened()) {
            mArduino.send(bytes);   //send given byte stream if serial port is open
        }
    }
}
