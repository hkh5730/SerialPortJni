package comma.hardware;

import android.util.Log;

import java.io.IOException;
import java.nio.ByteBuffer;

/**
 * Created by hkh on 18-2-27.
 */

public class SerialPort {
    static {
        System.loadLibrary("rk3399_serial_port");
        Log.d("SerialPort","loadLibrary rk3399_serial_port");
    }

    public void Close() {
        close();
    }


    public void Open(String path, int speed) throws IOException{
        open(path, speed);
    }


    public int Read(ByteBuffer buffer) throws IOException {
        if (buffer.isDirect()) {
            return readDirect(buffer, buffer.remaining());
        } else if (buffer.hasArray()) {
            return readArray(buffer.array(), buffer.remaining());
        } else {
            throw new IllegalArgumentException("buffer is not direct and has no array");
        }
    }

    public void Write(ByteBuffer buffer, int length)  throws IOException {
        if (buffer.isDirect()) {
            writeDirect(buffer, length);
        } else if (buffer.hasArray()) {
            writeArray(buffer.array(), length);
        } else {
            throw new IllegalArgumentException("buffer is not direct and has no array");
        }
    }
    private native void open(String path, int speed)throws IOException;
    private native void close();
    private native int readArray(byte[] buffer, int length) throws IOException;
    private native int readDirect(ByteBuffer buffer, int length) throws IOException;
    private native void writeArray(byte[] buffer, int length) throws IOException;
    private native void writeDirect(ByteBuffer buffer, int length) throws IOException;
}
