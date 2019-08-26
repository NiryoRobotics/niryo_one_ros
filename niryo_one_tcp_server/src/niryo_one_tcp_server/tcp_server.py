#!/usr/bin/env python

# tcp_server.py
# Copyright (C) 2019 Niryo
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
import rospy
import Queue
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
        self.__command_executor_thread = Thread(target=self.__command_executor_loop)
        self.__is_running = True
        self.__is_busy = False
        self.__client = None
        self.__interpreter = CommandInterpreter()
        self.__queue = Queue.Queue(1)

    def __del__(self):
        self.quit()

    def start(self):
        self.__loop_thread.start()
        self.__command_executor_thread.start()

    def quit(self):
        self.__is_running = False
        if self.__loop_thread.isAlive():
            self.__loop_thread.join()
        if self.__command_executor_thread.isAlive():
            self.__command_executor_thread.join()
        if self.__client is not None:
            self.__shutdown_client()
        self.__server.close()

    def __treat_command(self, command):
        result = self.__interpreter.interpret_command(command)
        self.__send(result)

    def __command_executor_loop(self):
        while self.__is_running is True:
            try:
                command_received = self.__queue.get(block=True, timeout=0.5)
                self.__is_busy = True
                self.__treat_command(command_received)
                self.__is_busy = False
            except Queue.Empty as e:
                pass

    def __answer_client_robot_busy(self, command_received):
        command_received_split = command_received.split(':', 1)
        if len(command_received_split) != 2:
            command_name = command_received
        else:
            command_name = command_received_split[0]
        self.__send(command_name + ":KO,Robot is busy right now, command ignored.")

    def __client_socket_event(self, inputs):
        command_received = self.__read_command()
        if command_received is not None:
            if self.__is_busy:
                self.__answer_client_robot_busy(command_received)
            else:
                self.__queue.put(command_received)
        else:
            rospy.loginfo("Client disconnect")
            self.__is_client_connected = False
            self.__shutdown_client()
            inputs.remove(self.__client)

    def __loop(self):
        rospy.loginfo("Tcp server started on port : " + str(self.__port))
        inputs = [self.__server]
        while self.__is_running is True:
            readable, writable, exceptional = select.select(inputs, [], [], 0.5)
            for s in readable:
                if self.__is_client_connected is False and s is self.__server:
                    self.__accept_client()
                    inputs.append(self.__client)
                elif s is not self.__server:
                    self.__client_socket_event(inputs)

    def __shutdown_client(self):
        if self.__client is not None:
            try:
                self.__client.shutdown(socket.SHUT_RDWR)
            except socket.error as e:
                pass
            self.__client.close()

    def __accept_client(self):
        self.__client, address = self.__server.accept()
        self.__is_client_connected = True
        rospy.loginfo("Client connected from ip address: " + str(address))

    def __read_command(self):
        READ_SIZE = 512
        try:
            received = self.__client.recv(READ_SIZE)
        except socket.error as e:
            rospy.loginfo(e)
            return None
        if not received:
            return None
        return received

    def __send(self, content):
        if self.__client is not None:
            try:
                self.__client.send(content)
            except socket.error as e:
                rospy.loginfo("Error while sending answer to client: " + str(e))
