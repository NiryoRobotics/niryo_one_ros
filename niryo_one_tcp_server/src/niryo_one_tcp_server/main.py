import signal
from tcp_server import TcpServer

if __name__ == "__main__":
    server = TcpServer()
    server.start()

    def signal_handler(sig, frame):
        print('You pressed Ctrl+C !')
        server.quit()

    signal.signal(signal.SIGINT, signal_handler)
    signal.pause()
