import socket
import threading
import time # Added for sleep in error case

class TCPSocketServer:
    def __init__(self, host, port, command_handler_callback, logger, rclpy_ok_check):
        self.host = host
        self.port = port
        self.command_handler = command_handler_callback
        self.logger = logger
        self.rclpy_ok_check = rclpy_ok_check # Function to check rclpy.ok()
        self.server_socket = None
        self.server_thread = None
        self.running = False

    def start(self):
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Allow address reuse
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1) # Listen for one connection at a time
            self.logger.info(f"TCP socket server listening on {self.host}:{self.port}")

            self.running = True
            self.server_thread = threading.Thread(target=self._server_loop, daemon=True)
            self.server_thread.start()
            return True
        except Exception as e:
            self.logger.error(f"Failed to initialize TCP socket server: {e}")
            self.running = False
            return False

    def _server_loop(self):
        while self.running and self.rclpy_ok_check():
            conn = None # Initialize conn here
            try:
                # self.logger.info("TCP server waiting for a connection...")
                # Set a timeout on accept to allow checking self.running periodically
                self.server_socket.settimeout(1.0) 
                try:
                    conn, addr = self.server_socket.accept()
                except socket.timeout:
                    continue # Go back to check self.running and rclpy.ok()
                
                self.server_socket.settimeout(None) # Reset timeout for connection operations

                self.logger.info(f"TCP connection accepted from {addr}")
                with conn:
                    while self.running and self.rclpy_ok_check():
                        data = conn.recv(1024)
                        if not data:
                            self.logger.info("TCP client disconnected.")
                            break
                        command = data.decode('utf-8').strip()
                        self.logger.info(f"Received command via TCP socket: '{command}'")
                        response_str = self.command_handler(command)
                        conn.sendall(response_str.encode('utf-8'))
            except socket.timeout: # Should not happen here if timeout is None
                self.logger.warn("Socket operation timed out unexpectedly on active connection.")
                continue
            except OSError as e:
                if self.running:
                    self.logger.error(f"TCP server OS error: {e}")
                break # Exit loop if socket is closed or major error
            except Exception as e:
                if self.running:
                    self.logger.error(f"TCP server loop error: {e}")
                if conn: # Try to close connection if an error occurred with it
                    try:
                        conn.close()
                    except Exception as ce:
                        self.logger.error(f"Error closing connection after exception: {ce}")
                time.sleep(1) # Avoid rapid looping on persistent errors
        self.logger.info("TCP socket server loop terminated.")
        if self.server_socket:
            try:
                self.server_socket.close() # Ensure socket is closed when loop ends
                self.logger.info("Main server socket closed from loop end.")
            except Exception as e:
                self.logger.error(f"Error closing main server socket in loop end: {e}")


    def stop(self):
        self.logger.info("Stopping TCP socket server...")
        self.running = False # Signal the server loop to stop
        
        # To unblock accept() if it's waiting, we can close the socket.
        # The loop will then exit due to OSError or self.running being false.
        if self.server_socket:
            try:
                self.server_socket.close() # This will cause accept() to raise an error
                self.logger.info("Server socket closed to interrupt accept().")
            except Exception as e:
                self.logger.error(f"Error closing server socket during stop: {e}")

        if self.server_thread and self.server_thread.is_alive():
            self.logger.info("Waiting for TCP server thread to join...")
            self.server_thread.join(timeout=2.0) # Wait for thread to finish
            if self.server_thread.is_alive():
                self.logger.warn("TCP server thread did not terminate cleanly.")
        self.logger.info("TCP socket server stopped.")

# Example usage (for testing socket_shell.py directly, not for ROS node)
if __name__ == '__main__':
    import logging
    # Basic logger for standalone testing
    test_logger = logging.getLogger("TestSocketServer")
    test_logger.setLevel(logging.INFO)
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
    test_logger.addHandler(handler)

    def dummy_command_handler(command):
        test_logger.info(f"Dummy handler received: {command}")
        if command == "ping":
            return "PONG_TCP\n"
        return f"ECHO_TCP: {command}\n"

    def rclpy_is_ok_dummy(): # Dummy for standalone testing
        return True

    # Test server
    # Use 0.0.0.0 to listen on all available interfaces, or 127.0.0.1 for localhost only
    server = TCPSocketServer(host="127.0.0.1", port=12345, 
                             command_handler_callback=dummy_command_handler, 
                             logger=test_logger,
                             rclpy_ok_check=rclpy_is_ok_dummy)
    if server.start():
        test_logger.info("Test server started. Press Ctrl+C to stop.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            test_logger.info("Ctrl+C received, stopping server.")
        finally:
            server.stop()
    else:
        test_logger.error("Failed to start test server.")