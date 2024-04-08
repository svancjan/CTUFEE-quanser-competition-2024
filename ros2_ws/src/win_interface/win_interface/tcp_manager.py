import zmq

class TCPPublisher:
    def __init__(self, port):
        self.context_pub = zmq.Context()

        self.out_sock = self.context_pub.socket(zmq.PUB)
        self.out_sock.bind("".join(("tcp://*:", str(port))))
        self.out_sock.setsockopt(zmq.SNDHWM, 1)

    def terminate(self):
        print("TCPManger will wait for threads to join...")
        self.out_sock.close()
        print("Sockets closed.")
        self.context_pub.term()
        print("TCPManager terminated.")

    def send_msg(self, msg):
        return self.out_sock.send_pyobj(msg)
    
class TCPSubscriber:
    def __init__(self, IP, port, topic=""):
        self.context_pull = zmq.Context()

        self.in_sock = self.context_pull.socket(zmq.SUB)
        self.in_sock.connect("".join(("tcp://", IP, ":", str(port))))
        self.in_sock.subscribe(topic)

    def terminate(self):
        print("TCPManger will wait for threads to join...")
        self.in_sock.close()
        print("Sockets closed.")
        self.context_pull.term()
        print("TCPManager terminated.")

    def receive_msg(self, timeout=10):
        try:
            return self.in_sock.recv_pyobj(flags=zmq.NOBLOCK)
        except zmq.Again as e:
            return None
