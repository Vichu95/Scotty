#!/usr/bin/env python3
import http.server
import socketserver
import os
import sys

def run_server(directory, port):
    # Change to the specified directory
    try:
        os.chdir(directory)
        print(f"Serving directory: {directory}")
    except FileNotFoundError:
        print(f"Error: Directory not found: {directory}")
        sys.exit(1)
    
    # Create handler
    handler = http.server.SimpleHTTPRequestHandler
    
    # Create server
    try:
        with socketserver.TCPServer(("", port), handler) as httpd:
            print(f"Server started at http://localhost:{port}")
            httpd.serve_forever()
    except OSError as e:
        print(f"Server error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nServer stopped by user")

if __name__ == "__main__":
    # Default values
    directory = "/home/roboubu/Documents/VishnuScooty/scotty_ws/src"
    port = 8080
    # default_directory = os.getcwd()
    # default_port = 8080
    
    # # Parse command-line arguments
    # directory = sys.argv[1] if len(sys.argv) > 1 else default_directory
    
    # if len(sys.argv) > 2:
    #     try:
    #         port = int(sys.argv[2])
    #     except ValueError:
    #         print(f"Error: Invalid port number: {sys.argv[2]}")
    #         sys.exit(1)
    # else:
    #     port = default_port
        
    # Run the server
    run_server(directory, port)

