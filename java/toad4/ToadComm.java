package toad4;

import java.io.IOException;

public interface ToadComm {
	static public class TimeoutException extends Exception {
	}

	static public class DeviceBusyException extends Exception {
	}

	static public class AccessDeniedException extends Exception {
	}

	static public class DeviceNotFound extends Exception {
	}

	boolean isConnected();

	void openConnection(String portName) throws DeviceBusyException, AccessDeniedException, DeviceNotFound, IOException;

	void closeConnection();

	int read(byte[] buffer, int len) throws Exception;

	void write(byte[] buffer, int len) throws Exception;

	void flush() throws Exception;

}