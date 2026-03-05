# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

# Import necessary modules
import ast
import math
import time
from multiprocessing import Pipe
from src.data.TrafficCommunication.useful.sharedMem import sharedMem
from src.templates.workerprocess import WorkerProcess
from src.templates.threadwithstop import ThreadWithStop
from src.data.TrafficCommunication.threads.threadTrafficCommunication import threadTrafficCommunication
from src.utils.messages.allMessages import CurrentSpeed, ImuData
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber

try:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
except Exception:
    rclpy = None
    Odometry = None
    Node = None


#   추가된 부분
#############################################################################
class threadTrafficDataCollector(ThreadWithStop):
    """Collect local vehicle state and store it in shared memory for server upload."""

    def __init__(self, shared_memory, queues_list, logger=None, debugging=False):
        super(threadTrafficDataCollector, self).__init__(pause=0.05)
        self.shared_memory = shared_memory
        self.queues_list = queues_list
        self.logger = logger
        self.debugging = debugging

        self.imu_subscriber = messageHandlerSubscriber(self.queues_list, ImuData, "lastOnly", True)
        self.speed_subscriber = messageHandlerSubscriber(self.queues_list, CurrentSpeed, "lastOnly", True)

        self.latest_pos = None
        self.latest_yaw = None
        self.latest_speed = None

        self._min_publish_period = 0.2  # seconds
        self._last_insert = {"devicePos": 0.0, "deviceRot": 0.0, "deviceSpeed": 0.0}

        self._ros_enabled = rclpy is not None and Odometry is not None and Node is not None
        self._ros_node = None
        self._ros_initialized_here = False
        self._next_ros_retry = 0.0

    def thread_work(self):
        self._poll_queue_state()
        self._spin_ros_once()
        self._flush_to_shared_memory()

    def stop(self):
        self._close_ros()
        super(threadTrafficDataCollector, self).stop()

    def _poll_queue_state(self):
        imu_payload = self.imu_subscriber.receive()
        if imu_payload is not None:
            yaw = self._extract_yaw(imu_payload)
            if yaw is not None:
                self.latest_yaw = yaw

        speed_payload = self.speed_subscriber.receive()
        if speed_payload is not None:
            try:
                self.latest_speed = float(speed_payload)
            except (TypeError, ValueError):
                pass

    def _extract_yaw(self, imu_payload):
        parsed = imu_payload
        if isinstance(imu_payload, str):
            try:
                parsed = ast.literal_eval(imu_payload)
            except (ValueError, SyntaxError):
                return None

        if not isinstance(parsed, dict):
            return None

        yaw = parsed.get("yaw", None)
        if yaw is None:
            return None

        try:
            return float(yaw)
        except (TypeError, ValueError):
            return None

    def _spin_ros_once(self):   # ROS에서 odom 메시지를 받아오는 부분
        if not self._ros_enabled:
            return

        now = time.monotonic()
        if self._ros_node is None:
            if now < self._next_ros_retry:
                return
            self._init_ros()
            return

        try:
            rclpy.spin_once(self._ros_node, timeout_sec=0.0)
        except Exception as exc:
            print(f"\033[1;97m[ Traffic Communication ] :\033[0m \033[1;93mWARNING\033[0m - /odom spin failed ({exc})")
            self._close_ros()
            self._next_ros_retry = time.monotonic() + 3.0

    def _init_ros(self):        # odom 구독
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
                self._ros_initialized_here = True
            self._ros_node = Node("traffic_com_odom_listener")
            self._ros_node.create_subscription(Odometry, "/odom", self._handle_odom, 10)
        except Exception as exc:
            print(f"\033[1;97m[ Traffic Communication ] :\033[0m \033[1;93mWARNING\033[0m - /odom listener init failed ({exc})")
            self._close_ros()
            self._next_ros_retry = time.monotonic() + 3.0

    def _close_ros(self):
        if self._ros_node is not None:
            try:
                self._ros_node.destroy_node()
            except Exception:
                pass
            self._ros_node = None

        if self._ros_initialized_here and rclpy is not None and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
            self._ros_initialized_here = False

    def _handle_odom(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        twist = msg.twist.twist

        self.latest_pos = (float(pos.x), float(pos.y))
        self.latest_yaw = self._quat_to_yaw(ori.x, ori.y, ori.z, ori.w)
        self.latest_speed = float(twist.linear.x)

    def _quat_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _flush_to_shared_memory(self):
        now = time.monotonic()

        if self.latest_pos is not None and (now - self._last_insert["devicePos"]) >= self._min_publish_period:
            self.shared_memory.insert("devicePos", [self.latest_pos[0], self.latest_pos[1]])
            self._last_insert["devicePos"] = now

        if self.latest_yaw is not None and (now - self._last_insert["deviceRot"]) >= self._min_publish_period:
            self.shared_memory.insert("deviceRot", [self.latest_yaw])
            self._last_insert["deviceRot"] = now

        if self.latest_speed is not None and (now - self._last_insert["deviceSpeed"]) >= self._min_publish_period:
            self.shared_memory.insert("deviceSpeed", [self.latest_speed])
            self._last_insert["deviceSpeed"] = now

##########################################################

class processTrafficCommunication(WorkerProcess):
    """This process receives the location of the car and sends it to the processGateway.
    
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Used for debugging.
        deviceID (int): The ID of the device.
        frequency (float): The frequency of communication.
    """

    # ====================================== INIT ==========================================
    def __init__(self, queueList, logging, deviceID, ready_event=None, debugging=False, frequency=1):
        self.queuesList = queueList
        self.logging = logging
        self.shared_memory = sharedMem()
        self.filename = "src/data/TrafficCommunication/useful/publickey_server_test.pem"
        self.deviceID = deviceID
        self.frequency = frequency
        self.debugging = debugging
        super(processTrafficCommunication, self).__init__(self.queuesList, ready_event)

    # ===================================== INIT TH ======================================
    def _init_threads(self):
        """Create the Traffic Communication thread and add it to the list of threads."""

        TrafficComTh = threadTrafficCommunication(
            self.shared_memory, self.queuesList, self.deviceID, self.frequency, self.filename
        )
        TrafficDataCollectorTh = threadTrafficDataCollector(
            self.shared_memory, self.queuesList, self.logging, self.debugging
        )
        self.threads.append(TrafficComTh)
        self.threads.append(TrafficDataCollectorTh)


# =================================== EXAMPLE =========================================
#             ++    THIS WILL RUN ONLY IF YOU RUN THE CODE FROM HERE  ++
#                  in terminal:    python3 processTrafficCommunication.py

if __name__ == "__main__":
    from multiprocessing import Queue
    import time

    shared_memory = sharedMem()
    locsysReceivePipe, locsysSendPipe = Pipe(duplex=False)
    queueList = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
    }
    # filename = "useful/publickey_server.pem"
    filename = "useful/publickey_server_test.pem"
    deviceID = 3
    frequency = 0.4
    traffic_communication = threadTrafficCommunication(
        shared_memory, queueList, deviceID, frequency, filename
    )
    traffic_communication.start()    

    start_time = time.time()
    duration = 10  # specify the duration in seconds
    
    shared_memory.insert("devicePos", [1.2, 2.3]) # send a position to the server
    shared_memory.insert("deviceRot", [3.4]) # send a rotation to the server
    shared_memory.insert("deviceSpeed", [4.5]) # send a speed to the server
    shared_memory.insert("historyData", [5.6, 6.7, 8]) # send a history data point to the server

    while time.time() - start_time < duration:
        try:
            print(queueList["General"].get(timeout=1))
        except:pass
    traffic_communication.stop()
