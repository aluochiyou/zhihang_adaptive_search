#!/usr/bin/env python3
"""Validation-only Gazebo truth relay. Never launch in formal mode.

It is isolated from static planning. The manager receives only a generic target
state topic and cannot infer target starts or static positions from this node.
"""
import json, rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
NS='/zhihang/search_v6'
class Relay:
    def __init__(self):
        rospy.init_node('validation_target_state_relay_v6')
        self.enabled=bool(rospy.get_param('/zhihang_search_v6/validation/truth_target_relay_enabled',False))
        self.targets=set(rospy.get_param('/zhihang_search_v6/perception/dynamic_targets',[]))
        self.mission_id=''
        self.pub=rospy.Publisher(f'{NS}/tracking/target_state',String,queue_size=100)
        rospy.Subscriber(f'{NS}/manager/status',String,self.status_cb,queue_size=1)
        if self.enabled:
            rospy.Subscriber('/gazebo/model_states',ModelStates,self.cb,queue_size=1)
            rospy.logwarn('VALIDATION ONLY: Gazebo target-state relay enabled; disable for formal mission')
        else:
            rospy.loginfo('truth relay disabled; formal visual tracker must publish target_state')
    def status_cb(self,msg):
        try:self.mission_id=str(json.loads(msg.data).get('mission_id',''))
        except Exception:pass
    def cb(self,msg):
        now=rospy.Time.now().to_sec()
        for i,name in enumerate(msg.name):
            if name not in self.targets:continue
            p=msg.pose[i].position;t=msg.twist[i].linear
            row={'mission_id':self.mission_id,'target_name':name,'position':[p.x,p.y,p.z],
                 'velocity':[t.x,t.y,t.z],'source_ros_time':now,'source':'validation_gazebo_truth_relay'}
            self.pub.publish(String(data=json.dumps(row,ensure_ascii=False)))
if __name__=='__main__':Relay();rospy.spin()
