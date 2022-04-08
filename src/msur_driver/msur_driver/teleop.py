import rclpy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# settings = termios.tcgetattr(sys.stdin)

msg = """
Reading from the keyboard and Publishing!
---------------------------
i - forward
, - backward
j - left
l - right
u - up
m - down
o - twist left
p - twist right

anything else : stop
t/b : increase/decrease max speeds by 10%
CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0, 0),
    ',': (-1, 0, 0, 0, 0),
    'j': (0, -1, 0, 0, 0),
    'l': (0, 1, 0, 0, 0),
    'u': (0, 0, 1, 0, 0),
    'm': (0, 0, -1, 0, 0),
    'o': (0, 0, 0, -1, 0),
    'p': (0, 0, 0, 1, 0),
}

speedBindings = {
    't': 1.1,
    'b': .9,
}


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    speed = 10
    x = 0.0
    y = 0.0
    z = 0.0
    w = 0.0

    try:
        print(msg)
        while True:
            key = getKey(settings)
            print(key)
            if key in moveBindings.keys():
                x = float(moveBindings[key][0])
                y = float(moveBindings[key][1])
                z = float(moveBindings[key][2])
                w = float(moveBindings[key][3])
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key]
                print(f'Updates speed: {speed}')

            else:
                x = y = z = 0.0
                if key == '\x03':
                    break

            twist = Twist()
            twist.linear.x = float(x * speed)
            twist.linear.y = float(y * speed)
            twist.linear.z = float(z * speed)
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = w * speed
            print(twist)
            pub.publish(twist)

    except Exception as e:
        raise e

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
