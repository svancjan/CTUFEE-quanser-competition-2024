import zmq
import pickle

class TCPManager:
    def __init__(self, IP, out_port, in_port):
        self.context_push = zmq.Context()
        self.context_pull = zmq.Context()
        self.IP = IP
        self.out_port = out_port
        self.in_port = in_port
    

        self.out_sock = self.context_push.socket(zmq.PUB)
        self.out_sock.bind("".join(("tcp://*:", str(self.out_port))))
        self.out_sock.setsockopt(zmq.SNDHWM, 1)

        self.in_sock = self.context_pull.socket(zmq.SUB)
        self.in_sock.connect("".join(("tcp://", self.IP, ":", str(self.in_port))))
        self.in_sock.subscribe("")

    def terminate(self):
        print("TCPManger will wait for threads to join...")
        self.out_sock.close()
        self.in_sock.close()
        print("Sockets closed.")
        self.context_push.term()
        self.context_pull.term()
        print("TCPManager terminated.")

    def receive_msg(self, timeout=10):
        try:
            return self.in_sock.recv_pyobj(flags=zmq.NOBLOCK)
        except zmq.Again as e:
            return None

    def send_msg(self, msg):
        return self.out_sock.send_pyobj(msg)

'''
class TCPManager:
    def __init__(self, IP, out_port, in_port):
        self.context_push = zmq.Context()
        self.context_pull = zmq.Context()
        self.IP = IP
        self.out_port = out_port
        self.in_port = in_port

        self.out_sock = self.context_push.socket(zmq.PUSH)
        self.out_sock.connect("".join(("tcp://", self.IP, ":", str(self.out_port))))
        self.out_sock.setsockopt(zmq.LINGER, 10) # wait 100 ms for messages to be sent before closing socket
        self.out_sock.setsockopt(zmq.SNDHWM, 1)

        self.in_sock = self.context_pull.socket(zmq.PULL)
        self.in_sock.bind("".join(("tcp://*:", str(self.in_port))))
        self.in_sock.setsockopt(zmq.LINGER, 10) # wait 100 ms for messages to be sent before closing socket

        self.in_poller = zmq.Poller()
        self.in_poller.register(self.in_sock, zmq.POLLIN)

    def terminate(self):
        print("TCPManger will wait for threads to join...")
        self.out_sock.close()
        self.in_sock.close()
        print("Sockets closed.")
        self.context_push.term()
        self.context_pull.term()
        print("TCPManager terminated.")

    def receive_msg(self, timeout=10):
        event = self.in_poller.poll(timeout)
        msg = None
        if event:
            msg = self.in_sock.recv_pyobj()
        #else:
        #    print("Timed out ({} ms) waiting for message...".format(timeout))
        return msg

    def send_msg(self, msg):
        self.out_sock.send_pyobj(msg, copy=True)
'''