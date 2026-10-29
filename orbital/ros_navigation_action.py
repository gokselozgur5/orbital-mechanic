"""Bounded Nav2 action lifecycle; no velocity path and no automatic motion arm.

The owning supervisor must arm the real guard, supply trusted health/stationary
feedback, gate all velocities and implement the stop callback. This component
proves action terminal ownership; a Nav2 success is not measured arrival.
"""
import math
import time
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from orbital.navigation import Target


class NavigationActionSession:
    def __init__(self, node, policy, waypoints, stop):
        if node.get_parameter('use_sim_time').value is not True:
            raise ValueError('Navigation action owner must use simulation time')
        self.node=node
        self.policy=policy
        self.waypoints=dict(waypoints)
        if set(self.waypoints)!=set(policy.waypoints):raise ValueError('Waypoint allowlists differ')
        if any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in self.waypoints.values()):
            raise ValueError('Finite named XY/yaw poses required')
        self.stop=stop
        self.client=ActionClient(node,NavigateToPose,'/navigate_to_pose')
        self.current=None
        self.handle=None
        self.state='idle'
        self.outcome=None
        self.deadline=None
        self.cancel_requested=False
        self.cancel_acknowledged=False
        self.cancel_future=None
        self.next_cancel=0.
        self.last_result=None
        self.clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.timer=node.create_timer(.05,self.tick,clock=self.clock)

    def request(self, waypoint):
        if self.current is not None:return None
        target=self.policy.request(waypoint,now=time.monotonic())
        if target is not None:
            self.current=target;self.state='waiting_for_arm';self.outcome=None
            self.cancel_requested=False;self.cancel_acknowledged=False
            self.handle=None;self.deadline=None;self.cancel_future=None
        return target

    def start(self, target, *, timeout=120.):
        now=time.monotonic()
        if (not isinstance(target,Target) or target!=self.current or self.state!='waiting_for_arm'
                or not self.policy.tick(now) or not self.policy.armed or self.policy.target!=target
                or self.node.get_clock().now().nanoseconds<=0):return False
        if type(timeout) not in (int,float) or not math.isfinite(timeout) or not .1<=timeout<=300:
            self.cancel(target,'invalid_goal_timeout')
            raise ValueError('Goal timeout must be in [0.1,300] seconds')
        if not self.client.server_is_ready():
            self.cancel(target,'action_server_unavailable')
            return False
        x,y,yaw=self.waypoints[target.waypoint]
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map'
        goal.pose.header.stamp=self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x=float(x);goal.pose.pose.position.y=float(y)
        goal.pose.pose.orientation.z=math.sin(yaw/2);goal.pose.pose.orientation.w=math.cos(yaw/2)
        self.deadline=now+timeout;self.state='dispatching'
        try:
            future=self.client.send_goal_async(goal)
            future.add_done_callback(lambda f:self._goal_response(target,f))
        except Exception:
            # Dispatch may have reached the server. Do not pretend it is terminal.
            self.cancel(target,'goal_dispatch_error')
            return False
        return True

    def cancel(self, target, reason='operator_cancel'):
        if target is None or target!=self.current or self.state=='terminal':return False
        was_waiting=self.state=='waiting_for_arm'
        self.cancel_requested=True
        if self.policy.target==target:self.policy.cancel(reason)
        self.stop()
        self.state='cancelling'
        if was_waiting:
            self._terminal(target,'not_dispatched',None)
        elif self.handle is not None:
            self._send_cancel()
        return True

    def _goal_response(self,target,future):
        if target!=self.current:return
        try:handle=future.result()
        except Exception:
            self.cancel(target,'goal_response_error');return
        if not handle.accepted:
            self._terminal(target,'rejected',None);return
        self.handle=handle
        try:
            result=handle.get_result_async()
            result.add_done_callback(lambda f:self._result(target,handle,f))
        except Exception:
            self.cancel(target,'goal_result_request_error');return
        if self.cancel_requested or self.policy.target!=target or not self.policy.tick(time.monotonic()):
            self.cancel(target,'invalidated_before_acceptance')
        else:self.state='running'

    def _send_cancel(self):
        now=time.monotonic()
        if self.handle is None or self.cancel_future is not None or now<self.next_cancel:return
        self.next_cancel=now+.5
        target=self.current;handle=self.handle
        try:
            self.cancel_future=handle.cancel_goal_async()
            self.cancel_future.add_done_callback(lambda f:self._cancel_response(target,handle,f))
        except Exception:
            self.cancel_future=None  # Remain stopped/cancelling; retry, never release.

    def _cancel_response(self,target,handle,future):
        if target!=self.current or handle is not self.handle:return
        self.cancel_future=None
        try:
            response=future.result()
            self.cancel_acknowledged=any(list(g.goal_id.uuid)==list(handle.goal_id.uuid) for g in response.goals_canceling)
        except Exception:self.cancel_acknowledged=False
        # This response only acknowledges cancellation intent. Await get_result.

    def _result(self,target,handle,future):
        if target!=self.current or handle is not self.handle:return
        try:status=future.result().status
        except Exception:
            self.cancel(target,'goal_result_error');return
        if status not in (GoalStatus.STATUS_SUCCEEDED,GoalStatus.STATUS_CANCELED,GoalStatus.STATUS_ABORTED):
            self.cancel(target,'nonterminal_result');return
        outcome=('nav2_succeeded_unverified' if status==GoalStatus.STATUS_SUCCEEDED and not self.cancel_requested
                 else 'cancelled' if status==GoalStatus.STATUS_CANCELED
                 else 'completed_after_cancel' if status==GoalStatus.STATUS_SUCCEEDED else 'aborted')
        self._terminal(target,outcome,status)

    def _terminal(self,target,outcome,status):
        if target!=self.current:return
        self.stop()
        if self.policy.target==target:
            if not self.policy.finish(target,now=time.monotonic()):self.policy.cancel('terminal_after_health_loss')
        if self.policy.cancel_pending==target:self.policy.confirm_terminal(target)
        self.state='terminal';self.outcome=outcome
        self.last_result={'target':target,'outcome':outcome,'nav2_status':status}

    def motion_permitted(self, target):
        """Evaluate deadlines on the command path, not only in a possibly late timer."""
        self.tick()
        return (target is not None and target==self.current and self.state=='running'
                and not self.cancel_requested and self.policy.target==target and self.policy.armed
                and self.deadline is not None and time.monotonic()<self.deadline)

    def release(self,target,*,stationary,stop_latched):
        """Caller supplies actual fresh feedback; no new goal before terminal + stop."""
        if (target!=self.current or self.state!='terminal' or stationary is not True or stop_latched is not True):return False
        self.current=None;self.handle=None;self.deadline=None;self.cancel_future=None;self.state='idle'
        return True

    def tick(self):
        self.policy.tick(time.monotonic())
        if self.current is None or self.state=='terminal':return
        if self.policy.target!=self.current or (self.state!='waiting_for_arm' and self.policy.stop_required):
            self.cancel(self.current,'localization_invalid')
        elif self.deadline is not None and time.monotonic()>=self.deadline:
            self.cancel(self.current,'goal_timeout')
        if self.cancel_requested and self.state!='terminal':self._send_cancel()

    def destroy(self):
        self.stop()
        self.node.destroy_timer(self.timer)
        self.client.destroy()
