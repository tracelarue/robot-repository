class FakeSerial:
    def __init__(self, port, baudrate, timeout=1):
        print(f"FakeSerial initialized on port {port} at {baudrate} baud.")
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

    def write(self, data):
        print("FakeSerial write:", data.decode('utf-8').strip())

    def readline(self):
        # Simulate a response from Arduino.
        return b"FakeSerial response\n"