package org.firstinspires.ftc.teamcode.NFS.RobotComponents;

public class StopwatchBrokenException extends RuntimeException {
    public StopwatchBrokenException() {
    }

    public StopwatchBrokenException(String message) {
        super("Stopwatch is broken " + message);
    }

    public StopwatchBrokenException(Throwable cause) {
        super(cause);
    }

    public StopwatchBrokenException(String message, Throwable cause) {
        super(message, cause);
    }

    public StopwatchBrokenException(String message, Throwable cause, boolean enableSuppression, boolean writableStackTrace) {
        super(message, cause, enableSuppression, writableStackTrace);
    }
}
