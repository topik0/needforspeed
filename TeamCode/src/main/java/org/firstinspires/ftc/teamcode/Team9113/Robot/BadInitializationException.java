package org.firstinspires.ftc.teamcode.Team9113.Robot;

public class BadInitializationException extends RuntimeException {
    public BadInitializationException() {
    }

    public BadInitializationException(String message) {
        super("Bad initialization detected: " + message);
    }

    public BadInitializationException(Throwable cause) {
        super(cause);
    }

    public BadInitializationException(String message, Throwable cause) {
        super(message, cause);
    }

    public BadInitializationException(String message, Throwable cause, boolean enableSuppression, boolean writableStackTrace) {
        super(message, cause, enableSuppression, writableStackTrace);
    }
}