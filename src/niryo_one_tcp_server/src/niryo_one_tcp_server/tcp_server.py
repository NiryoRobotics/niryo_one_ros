#!/usr/bin/env python

# tcp_server.py
# Copyright (C) 2017 Niryo
# All rights reserved.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import socket
import select
from threading import Thread
from command_interpreter import CommandInterpreter

class TcpServer:
    def __init__(self):
        self.__port = 40001
        self.__server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.__server.bind(('', self.__port))
        self.__server.listen(1)
        self.__is_client_connected = False
        self.__loop_thread = Thread(target=self.__loop)
        self.__is_running = True
        self.__client = None
        self.__interpreter = CommandInterpreter()

    def __del__(self):
        self.quit()
        pass

    def start(self):
        self.__loop_thread.start()
        pass

    def quit(self):
        self.__is_running = False
        if self.__loop_thread.isAlive():
            self.__loop_thread.join()
        self.__server.close()

    def __loop(self):
        print("Tcp server started on port : ", self.__port)
        inputs = [self.__server]
        while self.__is_running is True:
            readable, writable, exceptional = select.select(inputs, [], [], 0.5)
            for s in readable:
                if self.__is_client_connected is False and s is self.__server:
                    self.__accept_client()
                    inputs.append(self.__client)
                elif s is not self.__server:
                    ret = self.__read_command()
                    if ret is not None:
                        self.__treat_command(ret)
                    else:
                        self.__is_client_connected = False
                        self.__shutdown_client()
                        inputs.remove(self.__client)

    def __shutdown_client(self):
        if self.__client is not None:
            self.__client.shutdown(socket.SHUT_RDWR)
            self.__client.close()

    def __accept_client(self):
        self.__client, address = self.__server.accept()
        self.__is_client_connected = True
        print("Client connected from ip address: ", address)

    def __read_command(self):
        READ_SIZE = 512
        try:
            received = self.__client.recv(READ_SIZE)
        except socket.error as e:
            print(e)
            return None
        # Means client is disconnected
        if not received:
            return None
        print(received)
        return received

    def __send(self, content):
        if self.__client is not None:
            print("SEND: ", content)
            self.__client.send(content)

    def __treat_command(self, command):
        result = self.__interpreter.interpret_command(command)
        try:
            self.__send(result)
        except socket.error as e:
            print("Error while send answer to client: " + str(e))

# TO REMOVE BELOW AND REMOVE CHMOD EXECUTABLE

import signal

if __name__ == "__main__":
    server = TcpServer()
    server.start()

    def signal_handler(sig, frame):
        print('You pressed Ctrl+C !')
        server.quit()

    signal.signal(signal.SIGINT, signal_handler)

    signal.pause()
    print("END")
