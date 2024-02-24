from platformio.public import UnityTestRunner
# Use Serial2 rather than Serial for test output in this project

class CustomTestRunner(UnityTestRunner):

    UNITY_FRAMEWORK_CONFIG = dict(
        native=dict(
            code="""
#include <stdio.h>
void unityOutputStart(unsigned long baudrate) { (void) baudrate; }
void unityOutputChar(unsigned int c) { putchar(c); }
void unityOutputFlush(void) { fflush(stdout); }
void unityOutputComplete(void) { }
        """,
            language="c",
        ),
        arduino=dict(
            code="""
#include <Arduino.h>
void unityOutputStart(unsigned long baudrate) { Serial2.begin(baudrate); }
void unityOutputChar(unsigned int c) { Serial2.write(c); }
void unityOutputFlush(void) { Serial2.flush(); }
void unityOutputComplete(void) { Serial2.end(); }
        """,
            language="cpp",
        ),
        mbed=dict(
            code="""
#include <mbed.h>
#if MBED_MAJOR_VERSION == 6
UnbufferedSerial pc(USBTX, USBRX);
#else
RawSerial pc(USBTX, USBRX);
#endif
void unityOutputStart(unsigned long baudrate) { pc.baud(baudrate); }
void unityOutputChar(unsigned int c) {
#if MBED_MAJOR_VERSION == 6
    pc.write(&c, 1);
#else
    pc.putc(c);
#endif
}
void unityOutputFlush(void) { }
void unityOutputComplete(void) { }
        """,
            language="cpp",
        ),
        espidf=dict(
            code="""
#include <stdio.h>
void unityOutputStart(unsigned long baudrate) { (void) baudrate; }
void unityOutputChar(unsigned int c) { putchar(c); }
void unityOutputFlush(void) { fflush(stdout); }
void unityOutputComplete(void) { }
        """,
            language="c",
        ),
        zephyr=dict(
            code="""
#include <sys/printk.h>
void unityOutputStart(unsigned long baudrate) { (void) baudrate; }
void unityOutputChar(unsigned int c) { printk("%c", c); }
void unityOutputFlush(void) { }
void unityOutputComplete(void) { }
        """,
            language="c",
        ),
        legacy_custom_transport=dict(
            code="""
#include <unittest_transport.h>
void unityOutputStart(unsigned long baudrate) { unittest_uart_begin(); }
void unityOutputChar(unsigned int c) { unittest_uart_putchar(c); }
void unityOutputFlush(void) { unittest_uart_flush(); }
void unityOutputComplete(void) { unittest_uart_end(); }
        """,
            language="cpp",
        ),
    )