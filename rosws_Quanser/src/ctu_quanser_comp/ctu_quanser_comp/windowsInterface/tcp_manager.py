import zmq

class TCPPublisher:
    def __init__(self, port):
        self.context_pub = zmq.Context()

        self.out_sock = self.context_pub.socket(zmq.PUB)
        self.out_sock.bind("".join(("tcp://*:", str(port))))
        self.out_sock.setsockopt(zmq.SNDHWM,1)
        self.out_sock.setsockopt(zmq.CONFLATE,1)

    def __del__(self):
        self.out_sock.close()
        self.context_pub.term()

    def send_msg(self, msg):
        return self.out_sock.send(msg)
    
class TCPSubscriber:
    def __init__(self, IP, port, topic=""):
        self.context_pull = zmq.Context()

        self.in_sock = self.context_pull.socket(zmq.SUB)
        self.in_sock.connect("".join(("tcp://", IP, ":", str(port))))
        self.in_sock.subscribe(topic)
        self.in_sock.setsockopt(zmq.RCVHWM,1)
        self.in_sock.setsockopt(zmq.CONFLATE,1)

    def __del__(self):
        self.in_sock.close()
        self.context_pull.term()

    def receive_msg(self, timeout=10):
        try:
            return self.in_sock.recv(flags=zmq.NOBLOCK)
        except zmq.Again as e:
            return None
