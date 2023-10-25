import sys
import json
import numpy as np
import copy
from enum import IntEnum
from pathlib import Path

import paho.mqtt.client as mqtt


class JointType(IntEnum):
    NOSE = 0,
    NECK = 1,
    RIGHT_SHOULDER = 2,
    RIGHT_ELBOW = 3,
    RIGHT_WRIST = 4,
    LEFT_SHOULDER = 5,
    LEFT_ELBOW = 6,
    LEFT_WRIST = 7,
    MID_HIP = 8,
    RIGHT_HIP = 9,
    RIGHT_KNEE = 10,
    RIGHT_ANKLE = 11,
    LEFT_HIP = 12,
    LEFT_KNEE = 13,
    LEFT_ANKLE = 14,
    RIGHT_EYE = 15,
    LEFT_EYE = 16,
    RIGHT_EAR = 17,
    LEFT_EAR = 18,
    LEFT_BIG_TOE = 19,
    LEFT_SMALL_TOE = 20,
    LEFT_HEEL = 21,
    RIGHT_BIG_TOE = 22,
    RIGHT_SMALL_TOE = 23,
    RIGHT_HEEL = 24,


class Pose2TwinServer():
        
    def __init__(self):
        self.mqtt_client = mqtt.Client(client_id='pose2twin_server', protocol=mqtt.MQTTv311, clean_session=True)

        self.mqtt_client.on_connect = lambda client, userdata, c, d: {
            print("Connected to MQTT server")
        }
        
        self.mqtt_client.message_callback_add('pose2twin/subscribe', self.on_positional_stream_subscribe)
        self.mqtt_client.message_callback_add('pose2twin/unsubscribe', self.on_positional_stream_unsubscribe)
        
        self.clients = []

    def connect_to_mqtt(self, host):
        self.mqtt_client.connect(host)
        self.mqtt_client.loop_start()
        
        self.mqtt_client.subscribe('pose2twin/subscribe')
        self.mqtt_client.subscribe('pose2twin/unsubscribe')

    def on_positional_stream_subscribe(self, client, userdata, message):
        print(str(message.payload)[2:-1])
        request = json.loads(str(message.payload)[2:-1])
        
        self.clients.append({
            'topic_id': request['topic_id'],
            'forward': request['forward'],
            'right': request['right'],
            'up': request['up'],
            'unit_scale': request['unit_scale']
        })

    def on_positional_stream_unsubscribe(self, client, userdata, message):
        request = json.loads(str(message.payload)[2:-1])
        print(request)
        
        self.clients = [client for client in self.clients if client['topic_id'] != request['topic_id']]

    def submit_human_joint_positions(self, joints):
        # print(joints_json)
        
        # print(joints['MID_HIP'])
        
        for client in self.clients:
            client_joints = copy.deepcopy(joints)
            
            for joint_key, joint in client_joints.items():
                # Apply coordinate transforms
                transformed_joint = np.dot(joint[2], client['forward'])
                transformed_joint += np.dot(joint[0], client['right'])
                transformed_joint += np.dot(-joint[1], client['up'])
                
                # Apply unit scaling
                transformed_joint *= client['unit_scale']
                
                # Add confidence back to list and convert to python list for json serializing
                np.append(transformed_joint, joint[3])
                client_joints.update({joint_key: np.ndarray.tolist(transformed_joint)})
                # client_joints.append(np.asarray([* client['unit_scale'], joints[joint_index, 3]]))
            
            # print(client_joints)
                        
            joints_json = json.dumps(client_joints)
            self.mqtt_client.publish(f'pose2twin/stream/{client["topic_id"]}', joints_json, qos=0)

    def submit_human_joint_rotations(self):
        pass

# if __name__ == '__main__':
#     script_dir = Path(__file__).parent.resolve()
    
    
#     # temp = 0
#     # while True:
#     #     mqtt_client.publish("pose2twin/test", "this is a test")
        