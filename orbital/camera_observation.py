"""Bounded source-bound RGB evidence, with actual acquisition times preserved."""
from collections import deque
from dataclasses import dataclass
import json
import math
import xml.etree.ElementTree as ET

from orbital.collision_geometry import Transform, sdf_pose
from orbital.inspection_observation import InspectionPending
from orbital.machine_registry import read_source, digest
from orbital.proximity import Unavailable, timestamp, MAX_POSE_AGE_NS

MAX_IMAGE_BYTES = 1024*1024
MAX_SAMPLE_SKEW_NS = 100000000
OPTICAL_FRAME = 'camera_optical_frame'
CAMERA_TOPICS = {'/camera/image_raw':[('ros_gz_bridge','sensor_msgs/msg/Image')],
                 '/camera/camera_info':[('ros_gz_bridge','sensor_msgs/msg/CameraInfo')]}


def canonical(value):
    return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)


def only(element, path):
    values = element.findall(path)
    if len(values)!=1:
        raise ValueError('camera_source_shape_mismatch')
    return values[0]


def joint_pose(root, parent, child):
    joints = [j for j in root.findall('joint') if j.find('child') is not None and j.find('child').get('link')==child]
    if len(joints)!=1 or joints[0].get('type')!='fixed' or only(joints[0],'parent').get('link')!=parent:
        raise ValueError('camera_mount_source_mismatch')
    origin = only(joints[0],'origin')
    element = ET.Element('pose_source')
    ET.SubElement(element,'pose').text = origin.get('xyz','0 0 0')+' '+origin.get('rpy','0 0 0')
    return Transform.pose(sdf_pose(element))


def close_transform(actual, expected, tolerance=1e-5):
    return all(abs(a-b)<=tolerance for a,b in zip((*actual.center,*sum(actual.axes,())),
                                                  (*expected.center,*sum(expected.axes,()))))


@dataclass(frozen=True)
class CameraContract:
    registry: object
    width: int
    height: int
    hfov: float
    mount: Transform
    sources: tuple

    @classmethod
    def load(cls, registry):
        world_raw = read_source(registry.root,registry.world_path)
        if digest(world_raw)!=registry.world_sha256:
            raise ValueError('camera_world_source_changed')
        world = ET.fromstring(world_raw)
        sensor = only(world,"world/model[@name='maintenance_robot']/link[@name='base_link']/sensor[@type='camera']")
        if len(world.findall('.//sensor[@type="camera"]'))!=1:
            raise ValueError('ambiguous_camera_source')
        camera = only(sensor,'camera')
        width,height = (int(only(camera,'image/'+key).text) for key in ('width','height'))
        hfov = float(only(camera,'horizontal_fov').text)
        if (not 1<=width<=1024 or not 1<=height<=768 or width*height*3>MAX_IMAGE_BYTES
                or not math.isfinite(hfov) or not .01<hfov<math.pi
                or only(camera,'image/format').text!='R8G8B8'
                or only(sensor,'topic').text!='/camera/image_raw'
                or only(camera,'camera_info_topic').text!='/camera/camera_info'
                or only(sensor,'gz_frame_id').text!=OPTICAL_FRAME
                or float(only(sensor,'update_rate').text)!=5.
                or set(c.tag for c in camera)-{'horizontal_fov','image','clip','camera_info_topic'}):
            raise ValueError('unsupported_camera_source')
        urdf_raw = read_source(registry.root,'config/robot.urdf')
        urdf = ET.fromstring(urdf_raw)
        body = joint_pose(urdf,'base_footprint','base_link')
        sensor_mount = joint_pose(urdf,'base_link','camera_link')
        optical = joint_pose(urdf,'camera_link',OPTICAL_FRAME)
        optical_source = ET.fromstring('<p><pose>0 0 0 -1.5707963267948966 0 -1.5707963267948966</pose></p>')
        body_source = only(world,"world/model[@name='maintenance_robot']/link[@name='base_link']")
        if (not close_transform(body,Transform.pose(sdf_pose(body_source)))
                or not close_transform(sensor_mount,Transform.pose(sdf_pose(sensor)))
                or not close_transform(optical,Transform.pose(sdf_pose(optical_source)))):
            raise ValueError('camera_mount_source_mismatch')
        sources = (('config/robot.urdf',digest(urdf_raw)),
                   ('config/bridge.yaml',digest(read_source(registry.root,'config/bridge.yaml'))))
        return cls(registry,width,height,hfov,body.compose(sensor_mount).compose(optical),sources)

    def verify_current(self):
        self.registry.verify_current()
        if any(digest(read_source(self.registry.root,path))!=value for path,value in self.sources):
            raise Unavailable('camera_source_changed')


class CameraEvidence:
    """At most two received images and infos, plus one immutable selected pair."""
    def __init__(self, contract):
        self.contract = contract
        self.images,self.infos = deque(maxlen=2),deque(maxlen=2)
        self.last = {}
        self.failure = self.admitted = self.clock = self.wall = None
        self.calibration = self.selected = None

    def tick(self, now, sim_ns):
        if self.failure:
            raise Unavailable(self.failure)
        timestamp(sim_ns)
        if (type(now) not in (int,float) or not math.isfinite(now) or sim_ns<=0
                or (self.clock is not None and (sim_ns<self.clock or now<self.wall))):
            self.failure = 'camera_clock_invalid'
            raise Unavailable(self.failure)
        self.clock,self.wall = sim_ns,now
        if self.admitted is None:
            self.admitted = sim_ns

    def header(self, msg, kind, now, sim_ns):
        self.tick(now,sim_ns)
        sec,nsec = msg.header.stamp.sec,msg.header.stamp.nanosec
        if type(sec) is not int or type(nsec) is not int or sec<0 or not 0<=nsec<10**9:
            raise ValueError('camera_stamp_invalid')
        stamp = timestamp(sec*10**9+nsec)
        if (msg.header.frame_id!=OPTICAL_FRAME or not self.admitted<=stamp<=sim_ns
                or sim_ns-stamp>MAX_POSE_AGE_NS or stamp<=self.last.get(kind,0)):
            raise ValueError('camera_frame_time_or_order_mismatch')
        if (type(msg.width) is not int or type(msg.height) is not int
                or (msg.width,msg.height)!=(self.contract.width,self.contract.height)):
            raise ValueError('camera_dimensions_mismatch')
        self.last[kind] = stamp
        return stamp

    def image(self, msg, now, sim_ns):
        try:
            stamp = self.header(msg,'image',now,sim_ns)
            size = self.contract.width*self.contract.height*3
            if (msg.encoding!='rgb8' or type(msg.step) is not int or msg.step!=self.contract.width*3
                    or type(msg.is_bigendian) is not int or msg.is_bigendian!=0):
                raise ValueError('unsupported_camera_image_layout')
            view = memoryview(msg.data)
            if view.format!='B' or not view.c_contiguous or view.ndim!=1 or view.nbytes!=size:
                raise ValueError('camera_payload_size_mismatch')
            self.images.append((stamp,now,bytes(view)))
            return True
        except (ValueError,TypeError,AttributeError,OverflowError):
            self.failure = self.failure or 'invalid_camera_image'
            return False

    def info(self, msg, now, sim_ns):
        try:
            stamp = self.header(msg,'info',now,sim_ns)
            values = {}
            for key,count in (('k',9),('r',9),('p',12),('d',None)):
                source = getattr(msg,key)
                if len(source) not in ((count,) if count is not None else (0,5)):
                    raise ValueError('invalid_camera_calibration_shape')
                if any(type(v) not in (int,float) or not math.isfinite(v) or abs(v)>10000 for v in source):
                    raise ValueError('invalid_camera_calibration_number')
                values[key] = list(source)
            k,r,p,d = (values[key] for key in ('k','r','p','d'))
            focal = self.contract.width/(2*math.tan(self.contract.hfov/2))
            if (msg.distortion_model not in ('','plumb_bob') or any(abs(v)>1e-9 for v in d)
                    or not all(math.isclose(k[i],focal,rel_tol=.001) for i in (0,4))
                    or abs(k[2]-self.contract.width/2)>1 or abs(k[5]-self.contract.height/2)>1
                    or any(abs(k[i]-v)>1e-9 for i,v in ((1,0),(3,0),(6,0),(7,0),(8,1)))
                    or any(abs(a-b)>1e-9 for a,b in zip(r,(1,0,0,0,1,0,0,0,1)))
                    or any(abs(a-b)>1e-6 for a,b in zip(p,(k[0],0,k[2],0,0,k[4],k[5],0,0,0,1,0)))):
                raise ValueError('camera_calibration_mismatch')
            if any(type(v) is not int or v not in (0,1) for v in (msg.binning_x,msg.binning_y)):
                raise ValueError('unsupported_camera_binning')
            roi = {key:getattr(msg.roi,key) for key in ('x_offset','y_offset','height','width','do_rectify')}
            if (any(type(roi[k]) is not int for k in ('x_offset','y_offset','height','width'))
                    or roi['x_offset']!=0 or roi['y_offset']!=0 or roi['do_rectify'] is not False
                    or (roi['width'],roi['height']) not in ((0,0),(self.contract.width,self.contract.height))):
                raise ValueError('unsupported_camera_roi')
            values.update(width=msg.width,height=msg.height,distortion_model=msg.distortion_model,
                          binning_x=msg.binning_x,binning_y=msg.binning_y,roi=roi)
            encoded = canonical(values)
            if self.calibration is not None and encoded!=self.calibration:
                raise ValueError('camera_calibration_changed')
            self.calibration = encoded
            self.infos.append((stamp,now,encoded))
            return True
        except (ValueError,TypeError,AttributeError,OverflowError):
            self.failure = self.failure or 'invalid_camera_info'
            return False

    def select(self, sample_stamp, settled_stamp, now, sim_ns):
        self.tick(now,sim_ns)
        if self.selected is None:
            candidates = [(image,info) for image in self.images for info in self.infos
                          if image[0]==info[0] and image[0]>=settled_stamp
                          and abs(image[0]-sample_stamp)<=MAX_SAMPLE_SKEW_NS
                          and 0<=sim_ns-image[0]<=MAX_POSE_AGE_NS
                          and all(0<=now-value[1]<=.2 for value in (image,info))]
            if not candidates:
                raise InspectionPending('awaiting_time_matched_camera')
            image,info = min(candidates,key=lambda pair:abs(pair[0][0]-sample_stamp))
            self.selected = (image,info)
        self.check(now,sim_ns)
        return self.selected[0][0]

    def check(self, now, sim_ns):
        self.tick(now,sim_ns)
        if self.selected is None:
            raise InspectionPending('awaiting_camera_selection')
        if (not 0<=sim_ns-self.selected[0][0]<=MAX_POSE_AGE_NS
                or any(not 0<=now-value[1]<=.2 for value in self.selected)):
            self.failure = 'camera_observation_expired'
            raise Unavailable(self.failure)

    def describe(self, sample_stamp, now, sim_ns, robot, optical):
        self.check(now,sim_ns)
        stamp = self.selected[0][0]
        if (optical.header.frame_id!='map' or optical.child_frame_id!=OPTICAL_FRAME
                or type(optical.header.stamp.sec) is not int or type(optical.header.stamp.nanosec) is not int
                or not 0<=optical.header.stamp.nanosec<10**9
                or optical.header.stamp.sec*10**9+optical.header.stamp.nanosec!=stamp):
            raise Unavailable('camera_transform_time_or_frame_mismatch')
        p,q = optical.transform.translation,optical.transform.rotation
        pose = (p.x,p.y,p.z,q.x,q.y,q.z,q.w)
        if not close_transform(Transform.pose(pose),Transform.pose(robot['robot_pose_xyz_xyzw']).compose(self.contract.mount)):
            raise Unavailable('camera_transform_mount_mismatch')
        return {'stamp_ns':stamp,'frame':OPTICAL_FRAME,'encoding':'rgb8',
                'width':self.contract.width,'height':self.contract.height,'step':self.contract.width*3,
                'telemetry_offset_ns':stamp-sample_stamp,'offset_limit_ns':MAX_SAMPLE_SKEW_NS,
                'received_age_s':now-self.selected[0][1],'info_received_age_s':now-self.selected[1][1],
                'simulation_age_s':(sim_ns-stamp)/1e9,'camera_info':json.loads(self.selected[1][2]),
                'camera_pose_xyz_xyzw':list(pose),'robot_state':robot,
                'source_sha256':dict(self.contract.sources)}
