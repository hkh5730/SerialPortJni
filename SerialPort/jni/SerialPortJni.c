#include<stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include "jni.h"

static int fd;

static void jniThrowException(JNIEnv* env, const char* className, const char* msg) {
    jclass clazz = (*env)->FindClass(env, className);
    if (!clazz) {
        //ALOGE("Unable to find exception class %s", className);
        /* ClassNotFoundException now pending */
        return;
    }

    if ((*env)->ThrowNew(env, clazz, msg) != JNI_OK) {
        //ALOGE("Failed throwing '%s' '%s'", className, msg);
        /* an exception, most likely OOM, will now be pending */
    }
    (*env)->DeleteLocalRef(env, clazz);
}


/*
 * Class:     comma_hardware_SerialPort
 * Method:    open
 * Signature: (Ljava/lang/String;I)V
 */
void Java_comma_hardware_SerialPort_open(JNIEnv *env, jobject thiz, jstring path, jint speed)
{
    switch (speed) {
        case 50:
            speed = B50;
            break;
        case 75:
            speed = B75;
            break;
        case 110:
            speed = B110;
            break;
        case 134:
            speed = B134;
            break;
        case 150:
            speed = B150;
            break;
        case 200:
            speed = B200;
            break;
        case 300:
            speed = B300;
            break;
        case 600:
            speed = B600;
            break;
        case 1200:
            speed = B1200;
            break;
        case 1800:
            speed = B1800;
            break;
        case 2400:
            speed = B2400;
            break;
        case 4800:
            speed = B4800;
            break;
        case 9600:
            speed = B9600;
            break;
        case 19200:
            speed = B19200;
            break;
        case 38400:
            speed = B38400;
            break;
        case 57600:
            speed = B57600;
            break;
        case 115200:
            speed = B115200;
            break;
        case 230400:
            speed = B230400;
            break;
        case 460800:
            speed = B460800;
            break;
        case 500000:
            speed = B500000;
            break;
        case 576000:
            speed = B576000;
            break;
        case 921600:
            speed = B921600;
            break;
        case 1000000:
            speed = B1000000;
            break;
        case 1152000:
            speed = B1152000;
            break;
        case 1500000:
            speed = B1500000;
            break;
        case 2000000:
            speed = B2000000;
            break;
        case 2500000:
            speed = B2500000;
            break;
        case 3000000:
            speed = B3000000;
            break;
        case 3500000:
            speed = B3500000;
            break;
        case 4000000:
            speed = B4000000;
            break;
        default:
             jniThrowException(env, "java/lang/IllegalArgumentException",
                              "Unsupported serial port speed");
            return;
    }
    const char *pathStr = (*env)->GetStringUTFChars(env, path, NULL);
    
    fd = open(pathStr, O_RDWR | O_NOCTTY);
    if (fd < 0) {
         jniThrowException(env, "java/io/IOException", "Could not open serial port");
        (*env)->ReleaseStringUTFChars(env, path, pathStr);
        return ;
    }
    (*env)->ReleaseStringUTFChars(env, path, pathStr);

    //(*env)->SetIntField(env, thiz, field_context, fd);

    struct termios tio;
    if (tcgetattr(fd, &tio))
        memset(&tio, 0, sizeof(tio));

    tio.c_cflag =  speed | CS8 | CLOCAL | CREAD;
    // Disable output processing, including messing with end-of-line characters.
    tio.c_oflag &= ~OPOST;
    tio.c_iflag = IGNPAR;
    tio.c_lflag = 0; /* turn of CANON, ECHO*, etc */
    /* no timeout but request at least one character per read */
    tio.c_cc[VTIME] = 0;
    tio.c_cc[VMIN] = 1;
    tcsetattr(fd, TCSANOW, &tio);
    tcflush(fd, TCIFLUSH);   
}


/*
 * Class:     comma_hardware_SerialPort
 * Method:    close
 * Signature: ()V
 */
void Java_comma_hardware_SerialPort_close(JNIEnv *env, jobject thiz)
{
    //int fd = (*env)->GetIntField(env, thiz, field_context);
    close(fd);
    //(*env)->SetIntField(env, thiz, field_context, -1);
}

/*
 * Class:     comma_hardware_SerialPort
 * Method:    readArray
 * Signature: ([BI)I
 */
jint Java_comma_hardware_SerialPort_readArray(JNIEnv *env, jobject thiz, jbyteArray buffer, jint length)
{
    //int fd = (*env)->GetIntField(env, thiz, field_context);
    jbyte* buf = (jbyte *)malloc(length);
    if (!buf) {
        jniThrowException(env, "java/lang/OutOfMemoryError", NULL);
        return -1;
    }

    int ret = read(fd, buf, length);
    if (ret > 0) {
        // copy data from native buffer to Java buffer
        (*env)->SetByteArrayRegion(env, buffer, 0, ret, buf);
    }

    free(buf);
    
    if (ret < 0)
        jniThrowException(env, "java/io/IOException", NULL);
    return ret;
}

/*
 * Class:     comma_hardware_SerialPort
 * Method:    readDirect
 * Signature: (Ljava/nio/ByteBuffer;I)I
 */
jint Java_comma_hardware_SerialPort_readDirect(JNIEnv *env, jobject thiz, jobject buffer, jint length)
{
    //int fd = (*env)->GetIntField(env, thiz, field_context);

    jbyte* buf = (jbyte *)(*env)->GetDirectBufferAddress(env, buffer);
    if (!buf) {
        return -1;
    }

    int ret = read(fd, buf, length);
    if (ret < 0)
        jniThrowException(env, "java/io/IOException", NULL);
    return ret;
}

/*
 * Class:     comma_hardware_SerialPort
 * Method:    writeArray
 * Signature: ([BI)V
 */
void Java_comma_hardware_SerialPort_writeArray(JNIEnv *env, jobject thiz, jbyteArray buffer, jint length)
{
    //int fd = (*env)->GetIntField(env, thiz, field_context);
    jbyte* buf = (jbyte *)malloc(length);
    if (!buf) {
        return;
    }
    (*env)->GetByteArrayRegion(env, buffer, 0, length, buf);

    jint ret = write(fd, buf, length);
    free(buf);
    if (ret < 0)
        jniThrowException(env, "java/io/IOException", NULL);
}

/*
 * Class:     comma_hardware_SerialPort
 * Method:    writeDirect
 * Signature: (Ljava/nio/ByteBuffer;I)V
 */
void Java_comma_hardware_SerialPort_writeDirect(JNIEnv *env, jobject thiz, jobject buffer, jint length)
{
    //int fd = (*env)->GetIntField(env, thiz, field_context);

    jbyte* buf = (jbyte *)(*env)->GetDirectBufferAddress(env, buffer);
    if (!buf) {
        return;
    }
    int ret = write(fd, buf, length);
    if (ret < 0)
        jniThrowException(env, "java/io/IOException", NULL);
}