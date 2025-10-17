import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import argparse, random, math, time

from . import config as CONFIG

class ProximitySensor(Node):
    def __init__(self):
        super().__init__('proximity_sensor')
        self.get_logger().info("Proximity sensor node started.")
        self.publisher = self.create_publisher(Float64, CONFIG.TOPIC_PROXIMITY_STATUS, 100)
        self.timer = self.create_timer(0.001 * CONFIG.MEASUREMENT_PERIOD, self.measureProximity)
        self.proximity = Float64()  # in millimeters
        self.proximity.data = 0.0

        random.seed(42)  # for reproducibility
        self.proximityAmplitude = 1200.0
        self.noiseMagnitude = 50.0 
        self.proximityMean = self.proximityAmplitude/2.0
        self.proximityFrequency = 0.05  # in Hz
        self.startTime = time.time()
    
    def cosineProximity(self, t: float) -> float:
        noise = random.uniform(-self.noiseMagnitude, self.noiseMagnitude)
        return max(0.0, noise + self.proximityMean + self.proximityAmplitude*(math.sin(self.proximityFrequency*math.pi*t))/2.0)

    def measureProximity(self):
        self.proximity.data = self.cosineProximity(time.time() - self.startTime)
        self.publisher.publish(self.proximity)

def main(args=None):
    rclpy.init()
    proximitySensor = ProximitySensor()
    rclpy.spin(proximitySensor)
    proximitySensor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    argParser = argparse.ArgumentParser()
    argParser.add_argument(
        "--default",
        type=str,
        default="default",
        help="default placeholder argument.",
        )
    try:
        main()
    except Exception as e:
        print(f"Proximity sensor main() terminated with exception: {e}")