import copy
import unittest

from orbital.navigation_graph import BASE_TOPICS,NAV_TOPICS,NAV_ACTIONS,NAV_NODES,PARENTS,graph_errors


def valid(phase='before'):
    topics=dict(BASE_TOPICS)
    if phase=='configured':topics.update(NAV_TOPICS)
    return {'nodes':[{'name':name,'namespace':'/'} for name in
                     ['ros_gz_bridge','robot_state_publisher','motion_guard','map_server','amcl',
                      *(NAV_NODES if phase=='configured' else ())]],
            'publishers':{topic:[{'node':n,'namespace':'/','type':t} for n,t in owners] for topic,owners in topics.items()},
            'services':[{'name':p,'node':'motion_guard','namespace':'/','type':'std_srvs/srv/Trigger'}
                        for p in ('/motion/stop','/motion/reset_stop')],
            'parents':{child:[parent] for child,parent in PARENTS.items()},
            'actions':[{'name':path,'node':n,'namespace':'/','type':t} for path,(n,t) in NAV_ACTIONS.items()]
                      if phase=='configured' else []}


class NavigationGraphTests(unittest.TestCase):
    def test_valid_phases(self):
        for phase in ('before','configured'):
            self.assertEqual(graph_errors(valid(phase),phase),[])

    def test_duplicate_tf_endpoint_with_same_name(self):
        value=valid();value['publishers']['/tf'].append(copy.deepcopy(value['publishers']['/tf'][0]))
        self.assertIn('Publisher ownership mismatch: /tf',graph_errors(value,'before'))

    def test_action_server_under_another_namespace(self):
        value=valid();value['actions']=[{'name':'/other/navigate','node':'other','namespace':'/other','type':'nav2_msgs/action/NavigateToPose'}]
        self.assertIn('Navigation action ownership mismatch',graph_errors(value,'before'))

    def test_bypass_and_wrong_controller_type(self):
        value=valid('configured');value['publishers']['/cmd_vel']=value['publishers']['/navigation/raw_cmd_vel']
        value['publishers']['/navigation/raw_cmd_vel'][0]['type']='geometry_msgs/msg/Twist'
        errors=graph_errors(value,'configured')
        self.assertIn('Publisher ownership mismatch: /cmd_vel',errors)
        self.assertIn('Publisher ownership mismatch: /navigation/raw_cmd_vel',errors)

    def test_duplicate_node_cannot_hide_behind_shared_name(self):
        value=valid('configured');value['nodes'].append({'name':'bt_navigator','namespace':'/'})
        self.assertIn('Unexpected navigation node ownership: bt_navigator',graph_errors(value,'configured'))

    def test_conflicting_or_missing_tf_parent(self):
        value=valid();value['parents']['base_footprint'].append('map');del value['parents']['camera_link']
        errors=graph_errors(value,'before')
        self.assertIn('Missing or conflicting TF parent: base_footprint',errors)
        self.assertIn('Missing or conflicting TF parent: camera_link',errors)

    def test_competing_guard_service(self):
        value=valid();value['services'].append({'name':'/motion/reset_stop','node':'other','namespace':'/','type':'std_srvs/srv/Trigger'})
        self.assertIn('Guard service ownership mismatch: /motion/reset_stop',graph_errors(value,'before'))
