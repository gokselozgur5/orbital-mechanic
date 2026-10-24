"""Health-to-stop adapter. Never arms, dispatches targets or emits velocity."""
import json
import time
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger
from orbital.health_lease import HealthLease


class HealthInterlock(Node):
    def __init__(self, *, name='orbital_health_interlock', use_sim_time=False):
        super().__init__(name,parameter_overrides=[Parameter('use_sim_time',value=use_sim_time)])
        self.lease=HealthLease()
        self.stop_client=self.create_client(Trigger,'/motion/stop')
        self.health_sub=self.create_subscription(String,'/navigation/health',self.on_health,10)
        self.state_pub=self.create_publisher(String,'/navigation/interlock',10)
        self.future=None
        self.request_generation=None
        self.request_at=0.
        self.next_request=0.
        self.stop_requests=0
        self.stop_acks=0
        self.steady=Clock(clock_type=ClockType.STEADY_TIME)
        self.timer=self.create_timer(.05,self.tick,clock=self.steady)

    def on_health(self,msg):
        try:
            if len(msg.data)>4096:raise ValueError('oversized health report')
            report=json.loads(msg.data)
        except (ValueError,TypeError):
            self.lease.invalidate('malformed_health')
            return
        self.lease.observe(report,time.monotonic())

    def tick(self):
        now=time.monotonic()
        self.lease.tick(now)
        if len(self.get_publishers_info_by_topic('/navigation/health')) != 1:
            self.lease.invalidate('health_authority_ambiguous')
        if self.future is not None:
            if self.future.done():
                try:
                    response=self.future.result()
                    if response is not None and response.success:
                        if self.lease.acknowledge_stop(self.request_generation):self.stop_acks+=1
                    else:self.lease.invalidate('guard_stop_rejected')
                except Exception:
                    self.lease.invalidate('guard_stop_failed')
                self.future=None
            elif now-self.request_at>=.5:
                self.future.cancel()
                self.future=None
                self.lease.invalidate('guard_stop_timeout')
        if self.lease.stop_required and self.future is None and now>=self.next_request:
            self.next_request=now+.1
            if self.stop_client.service_is_ready():
                self.request_generation=self.lease.generation
                self.request_at=now
                self.future=self.stop_client.call_async(Trigger.Request())
                self.stop_requests+=1
        self.state_pub.publish(String(data=json.dumps({
            'healthy':self.lease.healthy,'stop_required':self.lease.stop_required,
            'reason':self.lease.reason,'generation':self.lease.generation,
            'source_run':self.lease.run_id,'source_epoch':self.lease.epoch,
            'stop_requests':self.stop_requests,'stop_acknowledgments':self.stop_acks})))
