import socket
import threading
import time

#first install opencv and python bindings, apt install python3-opencv
import cv2
import time

import drone_service.Tello as Tello

import rclpy
from rclpy.node import Node

from drone_interfaces.srv import Move
from std_srvs.srv import Empty
class VideoTello():
    def __init__(self):
        global _running
        self._running = True
        self.video = cv2.VideoCapture("udp://@0.0.0.0:11111")

    def terminate(self):
        self._running = False
        self.video.release()
        cv2.destroyAllWindows()

    def recv(self):
        """ Handler for Tello states message """
        global _running
        while self._running:
            try:
                ret, frame = self.video.read()
                if ret:
                    # Resize frame
                    height, width, _ = frame.shape
                    new_h = int(height / 2)
                    new_w = int(width / 2)

                    # Resize for improved performance
                    new_frame = cv2.resize(frame, (new_w, new_h))

                    # Display the resulting frame
                    cv2.imshow('Tello', new_frame)
                    cv2.imwrite('Snapshot.jpg', new_frame)
                # Wait for display image frame
                # cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.waitKey(1)
            except Exception as err:
                print(err)

class DroneServer(Node):
    drone_response = "no_response"

    def __init__(self):
        super().__init__('drone_server')

        self.local_ip = ''
        self.local_port = 8889
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # socket for sending cmd
        self.socket.bind((self.local_ip, self.local_port))

        # thread for receiving cmd ack
        self.receive_thread = threading.Thread(target=self._receive_thread)
        self.receive_thread.daemon = True
        self.receive_thread.start()

        self.tello_ip = '192.168.10.1'
        self.tello_port = 8889
        self.tello_address = (self.tello_ip, self.tello_port)
        self.MAX_TIME_OUT = 15.0


        self.srv = self.create_service(Move, 'move_forward', self.move_forward_callback)

        self.srv = self.create_service(Move, 'move_backward', self.move_forward_callback)
        self.srv = self.create_service(Move, 'move_left', self.move_left_callback)

        self.srv = self.create_service(Move, 'move_right', self.move_right_callback)

        self.srv = self.create_service(Empty, 'flip_forward', self.flip_forward_callback)
        self.srv = self.create_service(Empty, 'flip_backward', self.flip_backward_callback)

        self.srv = self.create_service(Empty, 'takeoff', self.takeoff_callback)
        self.srv = self.create_service(Empty, 'land', self.land_callback)
        self.srv = self.create_service(Empty, 'battery', self.battery_callback)
        self.srv = self.create_service(Empty, 'speed', self.speed_callback)
        self.srv = self.create_service(Empty, 'streamon', self.video_callback)
        self.srv = self.create_service(Empty, 'time', self.time_callback)


    def send_command(self, msg):
        command = msg #the actual command string

        self.get_logger().info('I heard: "%s"' % msg)
        self.socket.sendto(command.encode('utf-8'), self.tello_address)
        print('sending command: %s to %s' % (command, self.tello_ip))

        start = time.time()
        now = time.time()
        diff = now - start
        if diff > self.MAX_TIME_OUT:
            print('Max timeout exceeded... command %s' % command)
            return
        print('Done!!! sent command: %s to %s' % (command, self.tello_ip))



    def move_forward_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Move Forward: %dcm' % (request.distance))
        command = "forward %d" % request.distance
        print(command)
        self.send_command("command")
        time.sleep(2)
        self.send_command(command)
        time.sleep(2) #wait for the response
        response.result = drone_response
        return response

    def move_left_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Move Left: %dcm' % (request.distance))
        command = "left %d" % request.distance
        print(command)
        self.send_command("command")
        time.sleep(1)
        self.send_command(command)
        time.sleep(1) #wait for the response
        response.result = drone_response
        return response

    def move_right_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Move Right: %dcm' % (request.distance))
        command = "right %d" % request.distance
        print(command)
        self.send_command("command")
        time.sleep(1)
        self.send_command(command)
        time.sleep(1) #wait for the response
        response.result = drone_response
        return response

    def move_backward_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Move Backward: %dcm' % (request.distance))
        command = "backward %d" % request.distance
        print(command)
        self.send_command("command")
        time.sleep(1)
        self.send_command(command)
        time.sleep(1) #wait for the response
        response.result = drone_response
        return response

    def flip_forward_callback(self, request, response):
        self.get_logger().info('Incoming request: flip forward')
        command = "flip f"
        print(command)
        self.send_command(command)
        return response

    def flip_backward_callback(self, request, response):
        self.get_logger().info('Incoming request: flip backward')
        command = "flip b"
        print(command)
        self.send_command(command)
        return response


    def takeoff_callback(self, request, response):
        self.get_logger().info('Incoming request: Takeoff')
        command = "takeoff"
        print(command)
        self.send_command("command")
        time.sleep(2)
        self.send_command(command)
        return response

    def land_callback(self, request, response):
        self.get_logger().info('Incoming request: Land')
        command = "land"
        print(command)
        self.send_command("command")
        time.sleep(2)
        self.send_command(command)
        return response



    def battery_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Battery')
        command = "battery?"
        print(command)
        self.send_command("command")
        time.sleep(2)
        self.send_command(command)
        time.sleep(2)
        #battery = int(self.response.decode('utf-8'))
        print("Battery is : %s" % self.response.decode('utf-8'))
        return response #battery = self.send_command(command)



    def speed_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Speed')
        command = "speed?"
        print(command)
        self.send_command("command")
        time.sleep(2)
        self.send_command(command)
        time.sleep(2)
        #battery = int(self.response.decode('utf-8'))
        print("Speed is : %s" % self.response.decode('utf-8'))
        return response #battery = self.send_command(command)





    def time_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Speed')
        command = "time?"
        print(command)
        self.send_command("command")
        time.sleep(2)
        self.send_command(command)
        time.sleep(2)
        #battery = int(self.response.decode('utf-8'))
        print("Time is : %s" % self.response.decode('utf-8'))
        return response #battery = self.send_command(command)





        

    def video_callback(self, request, response):
        self.get_logger().info('Incoming request: streamon')
        #command = "streamon"
        #print(command)
        #self.send_command("command")
        #time.sleep(2)
        #self.send_command(command)
        #tello1 = Tello.Controller()
        print("sending command")
        self.send_command("command")
        #time.sleep(3)
        print("sending streamon")
        self.send_command("streamon")
        #time.sleep(3)

        print("starting video thread")
        t = VideoTello()
        recvThread = threading.Thread(target=t.recv)
        recvThread.start()
        print("video thread started")
        time.sleep(5)
        print("Snaphot")
        return response

    def _receive_thread(self):
        global drone_response
        #Listen to responses from the Tello.
        while True:
            try:
                self.response, ip = self.socket.recvfrom(1024)
                print('from %s: %s' % (ip, self.response))
                drone_response = str(self.response) #convert from byte string to string
            except (socket.error, exc):
                print("Caught exception socket.error : %s" % exc)



def main(args=None):
    rclpy.init(args=args)

    node = DroneServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - Done automatically when node is garbage collected)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
