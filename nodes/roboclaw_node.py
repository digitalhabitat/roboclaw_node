#!/usr/bin/env python
from math import pi, cos, sin
import swri_rospy
import diagnostic_msgs
import diagnostic_updater
from roboclaw_driver.roboclaw_driver import Roboclaw
import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry

__author__ = "bwbazemore@uga.edu (Brad Bazemore)"

# TODO need to find some better was of handling OSerror 11 or preventing it, any ideas?

class EncoderOdom:
    def __init__(self, ticks_per_meter, base_width, publish_tf, child_frame):
        self.TICKS_PER_METER = ticks_per_meter
        self.BASE_WIDTH = base_width
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.cur_x = 0
        self.cur_y = 0
        self.cur_theta = 0.0
        self.last_enc_left = 0
        self.last_enc_right = 0
        self.last_enc_time = rospy.Time.now()
        self.PUBLISH_TF = publish_tf
        self.CHILD_FRAME = child_frame  

    @staticmethod
    def normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def update(self, enc_left, enc_right):
        rospy.loginfo("inside update() funct line: 38") 
        left_ticks = enc_left - self.last_enc_left
        right_ticks = enc_right - self.last_enc_right
        self.last_enc_left = enc_left
        self.last_enc_right = enc_right

        dist_left = left_ticks / self.TICKS_PER_METER
        dist_right = right_ticks / self.TICKS_PER_METER
        dist = (dist_right + dist_left) / 2.0

        current_time = rospy.Time.now()
        d_time = (current_time - self.last_enc_time).to_sec()
        self.last_enc_time = current_time

        # TODO find better what to determine going straight, this means slight deviation is accounted
        if left_ticks == right_ticks:
            d_theta = 0.0
            self.cur_x += dist * cos(self.cur_theta)
            self.cur_y += dist * sin(self.cur_theta)
        else:
            d_theta = (dist_right - dist_left) / self.BASE_WIDTH
            r = dist / d_theta
            self.cur_x += r * (sin(d_theta + self.cur_theta) - sin(self.cur_theta))
            self.cur_y -= r * (cos(d_theta + self.cur_theta) - cos(self.cur_theta))
            self.cur_theta = self.normalize_angle(self.cur_theta + d_theta)

        if abs(d_time) < 0.000001:
            vel_x = 0.0
            vel_theta = 0.0
        else:
            vel_x = dist / d_time
            vel_theta = d_theta / d_time

        return vel_x, vel_theta

    def update_publish(self, enc_left, enc_right):
        if type(enc_left) is not int:
            rospy.logerr("enc_left : %s" % enc_left)
        if type(enc_right) is not int:
            rospy.logerr("enc_right : %s" % enc_right)
        
        # 2106 per 0.1 seconds is max speed, error in the 16th bit is 32768
        # TODO lets find a better way to deal with this error
        # Make sure this callback is ignored while reading encoders
        rospy.loginfo("inside update_publish() funct line: 74")
        if abs(enc_left - self.last_enc_left) > 20000:
            rospy.logerr("Ignoring left encoder jump: cur %d, last %d" % (enc_left, self.last_enc_left))
        elif abs(enc_right - self.last_enc_right) > 20000:
            rospy.logerr("Ignoring right encoder jump: cur %d, last %d" % (enc_right, self.last_enc_right))
        else:
            vel_x, vel_theta = self.update(enc_left, enc_right)
            self.publish_odom(self.cur_x, self.cur_y, self.cur_theta, vel_x, vel_theta)

    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vth):
        rospy.loginfo("inside publish_odom() funct line 93")
        quat = tf.transformations.quaternion_from_euler(0, 0, cur_theta)
        current_time = rospy.Time.now()
        
        if self.PUBLISH_TF == 1:       
            br = tf.TransformBroadcaster()
            br.sendTransform((cur_x, cur_y, 0),
                        tf.transformations.quaternion_from_euler(0, 0, -cur_theta),
                        current_time,
                        self.CHILD_FRAME,
                        "odom")

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'

        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*quat)

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 99999
        odom.pose.covariance[21] = 99999
        odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.01

        odom.child_frame_id = self.CHILD_FRAME
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance

        self.odom_pub.publish(odom)


class Node:
    
    def __init__(self):

        self.ERRORS = {0x0000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
                       0x0001: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
                       0x0002: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 over current"),
                       0x0004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Emergency Stop"),
                       0x0008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
                       0x0010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
                       0x0020: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main batt voltage high"),
                       0x0040: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage high"),
                       0x0080: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage low"),
                       0x0100: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 driver fault"),
                       0x0200: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 driver fault"),
                       0x0400: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage high"),
                       0x0800: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage low"),
                       0x1000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
                       0x2000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
                       0x4000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
                       0x8000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home")}

        rospy.init_node("roboclaw_node")
        rospy.on_shutdown(self.shutdown) # shutdown signal will trigger shutdown function
        rospy.loginfo("Connecting to roboclaw")
        dev_name = rospy.get_param("~dev", "/dev/ttyACM0")
        baud_rate = int(rospy.get_param("~baud", "115200"))
        self.address = int(rospy.get_param("~address", "128"))
        if self.address > 0x87 or self.address < 0x80:
            rospy.logfatal("Address out of range")
            rospy.signal_shutdown("Address out of range")

        self.rc = Roboclaw(dev_name, baud_rate)
        # TODO need someway to check if address is correct
        try:
            self.rc.Open()
        except Exception as e:
            rospy.logfatal("Could not connect to Roboclaw")
            rospy.logdebug(e)
            rospy.signal_shutdown("Could not connect to Roboclaw")

        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("Roboclaw")
        self.updater.add(diagnostic_updater.
                         FunctionDiagnosticTask("Vitals", self.check_vitals))

        try:
            version = self.rc.ReadVersion(self.address)
        except Exception as e:
            rospy.logwarn("Problem getting roboclaw version")
            rospy.logdebug(e)
            pass

        if not version[0]:
            rospy.logwarn("Could not get version from roboclaw")
        else:
            rospy.logdebug(repr(version[1]))

        self.rc.SpeedM1M2(self.address, 0, 0)
        self.rc.ResetEncoders(self.address)

        self.MAX_SPEED = float(rospy.get_param("~max_speed", "2.0"))
        self.TICKS_PER_METER = float(rospy.get_param("~tick_per_meter", "4342.2"))
        self.BASE_WIDTH = float(rospy.get_param("~base_width", "0.315"))
        self.TWIST_COMMAND = rospy.get_param("~twist_command", "roboclaw/cmd_vel")
        self.SINGLE_MOTOR = bool(rospy.get_param("~single_motor","false"))
        self.PUBLISH_TF = bool(rospy.get_param("~publish_tf","true"))
        self.CHILD_FRAME = rospy.get_param("~child_frame", "base_link") 

        self.encodm = EncoderOdom(self.TICKS_PER_METER, self.BASE_WIDTH, self.PUBLISH_TF, self.CHILD_FRAME )
        self.last_set_speed_time = rospy.get_rostime()
				
        # queue_size = 1 to make sure Roboclaw stops spining
				# this was an issue if cmd_vel frequency was too high 1000/sec
        # using swri_ropspy.Subscriber() swri_ropspy.Timer() for single threaded callbacks
        # multithreaded cmd_callback would eventually cause crash while the while-loop was reading encoders 
        # https://github.com/swri-robotics/marti_common/blob/master/swri_rospy/nodes/single_threaded_example
        swri_rospy.Subscriber(self.TWIST_COMMAND, Twist, self.cmd_vel_callback, queue_size = 1)
        swri_rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        rospy.sleep(1)

        rospy.logdebug("dev %s", dev_name)
        rospy.logdebug("baud %d", baud_rate)
        rospy.logdebug("address %d", self.address)
        rospy.logdebug("max_speed %f", self.MAX_SPEED)
        rospy.logdebug("ticks_per_meter %f", self.TICKS_PER_METER)
        rospy.logdebug("base_width %f", self.BASE_WIDTH)
        rospy.logdebug("twist_command %s", self.TWIST_COMMAND)
        rospy.logdebug("single_motor %d", self.SINGLE_MOTOR)
        rospy.logdebug("publsih_tf %d", self.PUBLISH_TF)
        rospy.logdebug("child_frame %s", self.CHILD_FRAME)

    def run(self):

        # TODO 
        rospy.loginfo("Starting motor drive")
        r_time = rospy.Rate(10)

        # see timer_callback()
        # spin() processes the callback queue until the node is stopped
        #   this replaces the <while not rospy.is_shutdown()> loop now that reading encoders is
        #   implemented in timer_callback()
        rospy.spin()
        """
        while not rospy.is_shutdown():

            if (rospy.get_rostime() - self.last_set_speed_time).to_sec() > 1:
                rospy.logwarn("Did not get command for 1 second, stopping")
                try:
                    self.rc.ForwardM1(self.address, 0)
                    self.rc.ForwardM2(self.address, 0)
                except OSError as e:
                    rospy.logerr("Could not stop")
                    rospy.logdebug(e)

            
            # Roboclaw_node will fail reading ecnoders if cmd_vel is spaming the node
            # Make a service to request cmd vel after read enc1 and enc2 and odom publish

            # TODO need find solution to the OSError11 looks like sync problem with serial
            status1, enc1, crc1 = None, None, None
            status2, enc2, crc2 = None, None, None
            
            
            # read encoder 1 *************************************************
            try:    
                status1, enc1, crc1 = self.rc.ReadEncM1(self.address)
                rospy.logwarn("status1 :  %s" % status1)
                rospy.logwarn("enc1    :  %s" % enc1)
                rospy.logwarn("crc1    :  %s" % crc1)

                if type(enc1) is not int:
                    rospy.logwarn("enc1 is not integer, self.rc.ReadEncM1() not reading properly")
            except ValueError:
                rospy.logwarn("enc2 value error")
                pass
            except OSError as e:
                rospy.logwarn("ReadEncM1 OSError: %d", e.errno)
                rospy.logdebug(e)
            
            # read encoder 2 *************************************************
            try:
                status2, enc2, crc2 = self.rc.ReadEncM2(self.address)
                rospy.logwarn("status2 :  %s" % status2)
                rospy.logwarn("enc2    :  %s" % enc2)
                rospy.logwarn("crc2    :  %s" % crc2)
                if type(enc2) is not int:
                    rospy.logwarn("enc2 is not integer, self.rc.ReadEncM1() not reading properly")
            except ValueError:
                rospy.logwarn("enc2 value error")
                pass
            except OSError as e:
                rospy.logwarn("ReadEncM2 OSError: %d", e.errno)
                rospy.logdebug(e)

            # update odom *************************************************
            if ('enc1' in vars()) and ('enc2' in vars()):
                rospy.logdebug(" Encoders %s %s" % (enc1, enc2))
                
                print 'updating odom'
                self.encodm.update_publish(enc1, enc2)

                self.updater.update()
            else:
                print 'encoder not in vars'
            """
               
            # request cmd_vel before sleep**************************************
            #r_time.sleep()

    def timer_callback(self, event):
        # orignal <while not rospy.is_shutdown():> loop tobe rewritten here
        #rospy.loginfo(rospy.get_caller_id() + " timer_callback() started")
        # stop motors if last twist topic recieved is older than 1 sec 
        if (rospy.get_rostime() - self.last_set_speed_time).to_sec() > 1:
            rospy.logwarn("Did not get command for 1 second, stopping")
            try:
                self.rc.ForwardM1(self.address, 0)
                self.rc.ForwardM2(self.address, 0)
            except OSError as e:
                rospy.logerr("Could not stop")
                rospy.logdebug(e)

        # TODO need find solution to the OSError11 looks like sync problem with serial
        status1, enc1, crc1 = None, None, None
        status2, enc2, crc2 = None, None, None

        # read encoder 1 *************************************************
        try:    
            status1, enc1, crc1 = self.rc.ReadEncM1(self.address)
            #rospy.logwarn("status1 :  %s" % status1)
            #rospy.logwarn("enc1    :  %s" % enc1)
            #rospy.logwarn("crc1    :  %s" % crc1)
            #if type(enc1) is not int:
            #    rospy.logwarn("enc1 is not integer, self.rc.ReadEncM1() not reading properly")
        except ValueError:
            rospy.logwarn("enc2 value error")
            pass
        except OSError as e:
            rospy.logwarn("ReadEncM1 OSError: %d", e.errno)
            rospy.logdebug(e)
         
        # read encoder 2 *************************************************
        try:
            status2, enc2, crc2 = self.rc.ReadEncM2(self.address)
            #rospy.logwarn("status2 :  %s" % status2)
            #rospy.logwarn("enc2    :  %s" % enc2)
            #rospy.logwarn("crc2    :  %s" % crc2)
            #if type(enc2) is not int:
            #    rospy.logwarn("enc2 is not integer, self.rc.ReadEncM1() not reading properly")
        except ValueError:
            rospy.logwarn("enc2 value error")
            pass
        except OSError as e:
            rospy.logwarn("ReadEncM2 OSError: %d", e.errno)
            rospy.logdebug(e)

        # update odom *************************************************
        if ('enc1' in vars()) and ('enc2' in vars()):
            rospy.logdebug(" Encoders %s %s" % (enc1, enc2))
               
            print 'updating odom'
            self.encodm.update_publish(enc1, enc2)
            self.updater.update()
        else:
            print 'encoder not in vars'

        #rospy.loginfo(rospy.get_caller_id() + "                       timer_callback() done")
            

    def cmd_vel_callback(self, twist):
        # Make sure this callback only occurs after reading encoders  
        # and only once or not at all
        # rospy.logwarn("cmd_vel_callback ************ ENTER")  
        self.last_set_speed_time = rospy.get_rostime()   

        linear_x = twist.linear.x
        if linear_x > self.MAX_SPEED:
            linear_x = self.MAX_SPEED
        if linear_x < -self.MAX_SPEED:
            linear_x = -self.MAX_SPEED

        # if Dual motor differential driver mode
        if self.SINGLE_MOTOR  == 0:
            vr = linear_x + twist.angular.z * self.BASE_WIDTH / 2.0  # m/s
            vl = linear_x - twist.angular.z * self.BASE_WIDTH / 2.0

            vr_ticks = int(vr * self.TICKS_PER_METER)  # ticks/s
            vl_ticks = int(vl * self.TICKS_PER_METER)

            rospy.logdebug("vr_ticks:%d vl_ticks: %d", vr_ticks, vl_ticks)

            try:
                # This is a hack way to keep a poorly tuned PID from making noise at speed 0
                if vr_ticks is 0 and vl_ticks is 0:
                    status1 = self.rc.ForwardM1(self.address, 0)
                    status2 = self.rc.ForwardM2(self.address, 0)
                    rospy.logwarn("cmd_vel_callback: status1 :  %s" % status1)
                    rospy.logwarn("cmd_vel_callback: status2 :  %s" % status2)
                else:
                    status1 = self.rc.SpeedM1M2(self.address, vr_ticks, vl_ticks)
                    rospy.logwarn("cmd_vel_callback: status1 :  %s" % status1)
            
            except ValueError:
                rospy.logwarn("tick value error")
                pass
            except OSError as e:
                rospy.logwarn("SpeedM1M2 OSError: %d", e.errno)
                rospy.logdebug(e)

        # if Single motor driver mode using M1
        else:
            vx_ticks = int(linear_x * self.TICKS_PER_METER) # ticks/s

            rospy.logdebug("vx_ticks:%d", vx_ticks)

            try:
                # This is a hack way to keep a poorly tuned PID from making noise at speed 0 
                if vx_ticks is 0:
                    status1 =  self.rc.ForwardM1(self.address, 0)
                    rospy.logwarn("cmd_vel_callback: status1 :  %s" % status1)
                else:
                    status1 = self.rc.SpeedM1(self.address, vx_ticks)
                    rospy.logwarn("cmd_vel_callback: status1 :  %s" % status1)

            except ValueError:
                rospy.logwarn("tick value error")
                pass
            except OSError as e:
            rospy.logwarn("SpeedM1 OSError: %d", e.errno)
            rospy.logdebug(e)
                
        rospy.logwarn("cmd_vel_callback ************ EXIT") 

    # TODO: Need to make this work when more than one error is raised
    def check_vitals(self, stat):
        try:
            status = self.rc.ReadError(self.address)[1]
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
            return
        state, message = self.ERRORS[status]
        stat.summary(state, message)
        try:
            stat.add("Main Batt V:", float(self.rc.ReadMainBatteryVoltage(self.address)[1] / 10))
            stat.add("Logic Batt V:", float(self.rc.ReadLogicBatteryVoltage(self.address)[1] / 10))
            stat.add("Temp1 C:", float(self.rc.ReadTemp(self.address)[1] / 10))
            stat.add("Temp2 C:", float(self.rc.ReadTemp2(self.address)[1] / 10))
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
        return stat

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self):
        rospy.logwarn("Shutting down")
        try:
            self.rc.ForwardM1(self.address, 0)
            self.rc.ForwardM2(self.address, 0)
        except OSError:
            rospy.logerr("Shutdown did not work trying again")
            try:
                self.rc.ForwardM1(self.address, 0)
                self.rc.ForwardM2(self.address, 0)
            except OSError as e:
                rospy.logerr("Could not shutdown motors!!!!")
                rospy.logdebug(e)


if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.logwarn("Exiting")
