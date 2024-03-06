import json
import zmq
import threading
import multiprocessing
import numpy as np

import time
import sys


class TCPManager:
    def __init__(self, IP, out_port, in_port):
        self.context_push = zmq.Context()
        self.context_pull = zmq.Context()
        self.IP = IP
        self.out_port = out_port
        self.in_port = in_port

        self.out_sock = self.context_push.socket(zmq.PUSH)
        self.out_sock.connect("".join(("tcp://", self.IP, ":", str(self.out_port))))

        self.in_sock = self.context_pull.socket(zmq.PULL)
        self.in_sock.bind("".join(("tcp://*:", str(self.in_port))))

        self.in_poller = zmq.Poller()
        self.in_poller.register(self.in_sock, zmq.POLLIN)

        self.latest_data = dict()
        self.run = True
        self.recv_thread = threading.Thread(target=self.receive_msgs, daemon=True, args=(self, ))

    def stop(self):
        print("TCPManger will wait for threads to join...")
        self.run = False
        self.out_sock.close()
        self.in_sock.close()
        self.context_push.term()
        self.context_pull.term()
        print("TCPManager terminated.")

    def start_receiving(self):
        print("TCPManager will start receiving thread...")
        self.recv_thread.start()
        print("Thread started")

    def receive_msgs(self, obj):
        while obj.run:
            event = self.in_poller.poll(100)
            if event:
                msg = self.in_sock.recv_pyobj()
                obj.latest_data = msg

    def receive_msg(self, timeout=100):
        event = self.in_poller.poll(timeout)
        if event:
            msg = self.in_sock.recv()
            payload = json.loads(msg)
        else:
            print("Timed out ({} ms) waiting for message...".format(timeout))

    def send_msg(self, msg):
        self.out_sock.send_pyobj(msg)

    def json_serialize(self, obj):
        if isinstance(obj, np.ndarray):
            # Ma takto cca 2.5x vetsi velikost jako string
            return str(obj.tobytes())
        return obj

