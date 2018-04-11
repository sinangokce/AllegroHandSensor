import numpy as np
import roslib
roslib.load_manifest('DataWrapper')
import rospy
from DataWrapper.msg import LasaDataStreamWrapper
import matplotlib.pyplot as mplt
import threading
import copy
import utils

vision_data_pos=np.array([0., 0., 0.])
vision_data_ori=np.array([0., 0., 0., 0.])
b_updated = False

pos_queue = []
seq_len = 100
#b_update_fig = False

lock = threading.Lock()

def from_frame_to_pen_tip(mark_pos, mark_rot_quat):
    mark_rot_mat = np.eye(3)
    utils.quaternion_to_R(mark_rot_quat, mark_rot_mat)
    return mark_pos + mark_rot_mat.dot(np.array([0, -0.145, 0]))
#    return mark_pos

def data_stream_callback(msg):
    with lock:
        global vision_data_pos, vision_data_ori, b_updated
#    print 'Callback called' 
#    if b_updated:
#        return
#    print msg.obj_pose.translation
        vision_data_pos[0] = msg.obj_pose.translation.x
        vision_data_pos[1] = msg.obj_pose.translation.y
        vision_data_pos[2] = msg.obj_pose.translation.z
        vision_data_ori[0] = msg.obj_pose.rotation.x
        vision_data_ori[1] = msg.obj_pose.rotation.y
        vision_data_ori[2] = msg.obj_pose.rotation.z
        vision_data_ori[3] = msg.obj_pose.rotation.w
        
        tip_pos = from_frame_to_pen_tip(vision_data_pos, vision_data_ori)
        #if zero, initialize with same value
        #NOTE that list.append not append a clone object!!!
        if(len(pos_queue) <= seq_len):
            pos_queue.append(copy.copy(tip_pos))
        else:
#            print 'Update queue'
            #not update only when motion is significant
            #if np.linalg.norm(pos_queue[-1] - vision_data_pos) > 1e-3:
            pos_queue.pop(0)
            pos_queue.append(copy.copy(tip_pos))
 
        b_updated = True
    return

def main():
    global vision_data_pos, vision_data_ori, b_updated
    global pos_queue, seq_len
    h = None
    ax = None
    n_counter = 0
#
    rospy.init_node('vision_test')
    r = rospy.Rate(50)
    sub = rospy.Subscriber('LasaDataStream', LasaDataStreamWrapper, data_stream_callback)
    mplt.ion()

    while not rospy.is_shutdown():
        with lock:
            if(b_updated == True):
                
#               print vision_data_pos
#               print type(vision_data_ori)
                b_updated = False
                #draw
                pos_queue_draw = np.array(pos_queue)
#                print pos_queue
                #print pos_queue_draw
                if h == None:
                    h, = mplt.plot(pos_queue_draw[:, 0], pos_queue_draw[:, 2], 'b.')
                    mplt.axis([2.2-0.3, 2.2+0.3, 0.4-0.3, 0.4+0.3])
                else:
                    #print pos_queue_draw[0, 0], pos_queue_draw[0, 1]
#                   print pos_queue_draw.shape
#                   print pos_queue_draw
#                   if n_counter > 1:
                    h.set_xdata(pos_queue_draw[:, 0])
                    h.set_ydata(pos_queue_draw[:, 2])
#                    ax.relim()
#                    ax.autoscale_view()
                    mplt.draw()
#                    n_counter = 0
                
#                print pos_queue
#                raw_input('press key...\n')
#        n_counter += 1
        
        r.sleep()
    return None

    
if __name__ == '__main__':
    main()
