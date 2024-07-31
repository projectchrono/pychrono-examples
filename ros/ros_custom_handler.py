import pychrono as ch
import pychrono.ros as chros

import rclpy.publisher
from std_msgs.msg import Int64


class MyCustomHandler(chros.ChROSHandler):
    """This custom handler will just publish int messages to a topic."""

    def __init__(self, topic):
        super().__init__(1)  # 1 Hz

        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None

        self.ticker = 0

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        print(f"Creating publisher for topic {self.topic} ...")
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)

        return True

    def Tick(self, time: float):
        print(f"Publishing {self.ticker} ...")
        msg = Int64()
        msg.data = self.ticker
        self.publisher.publish(msg)
        self.ticker += 1


def main():
    # Create system
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))

    # add a floor, box and sphere to the scene
    phys_mat = ch.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)

    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVector3d(0, 0, -1))
    floor.SetFixed(True)
    floor.SetName("base_link")
    sys.Add(floor)

    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))
    box.SetName("box")
    sys.Add(box)

    # Create ROS manager
    ros_manager = chros.ChROSPythonManager()
    # create simulation clock handler
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    # create body handler for the floor
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box"))
    # create tf handler for the box
    tf_handler = chros.ChROSTFHandler(30)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)
    # Create the custom handler
    custom_handler = MyCustomHandler("~/my_topic")
    ros_manager.RegisterPythonHandler(custom_handler)

    ros_manager.Initialize()

    # Simulation loop
    time = 0
    time_step = 1e-3
    time_end = 30

    realtime_timer = ch.ChRealtimeStepTimer()
    while time < time_end:
        sys.DoStepDynamics(time_step)
        time = sys.GetChTime()

        if not ros_manager.Update(time, time_step):
            break

        realtime_timer.Spin(time_step)


if __name__ == "__main__":
    main()
