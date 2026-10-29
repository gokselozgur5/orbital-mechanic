import copy
import math
from pathlib import Path
import shutil
import tracemalloc
from types import SimpleNamespace as NS

from orbital.camera_observation import CameraContract,CameraEvidence,MAX_IMAGE_BYTES
from orbital.inspection_observation import InspectionPending
from orbital.machine_registry import MachineRegistry
from test_machine_registry import RegistryFixture

ROOT=Path(__file__).resolve().parents[1]


def messages(stamp=1500000000):
    header=NS(frame_id='camera_optical_frame',stamp=NS(sec=stamp//10**9,nanosec=stamp%10**9))
    focal=640/(2*math.tan(.6))
    return (NS(header=copy.deepcopy(header),width=640,height=480,step=1920,encoding='rgb8',is_bigendian=0,
               data=bytearray((7,31,193))*(640*480)),
            NS(header=copy.deepcopy(header),width=640,height=480,distortion_model='plumb_bob',
               k=[focal,0.,320.,0.,focal,240.,0.,0.,1.],r=[1.,0.,0.,0.,1.,0.,0.,0.,1.],
               p=[focal,0.,320.,0.,0.,focal,240.,0.,0.,0.,1.,0.],d=[0.]*5,binning_x=0,binning_y=0,
               roi=NS(x_offset=0,y_offset=0,width=0,height=0,do_rectify=False)))


class CameraObservationTests(RegistryFixture):
    def setUp(self):
        super().setUp()
        for name in ('config/robot.urdf','config/bridge.yaml'):shutil.copyfile(ROOT/name,self.root/name)
        self.registry=MachineRegistry.load(self.root)
        self.contract=CameraContract.load(self.registry)

    def ready(self):
        evidence=CameraEvidence(self.contract);evidence.tick(10.,10**9)
        image,info=messages()
        self.assertTrue(evidence.image(image,10.5,1500000000))
        self.assertTrue(evidence.info(info,10.5,1500000000))
        evidence.select(1550000000,1400000000,10.55,1550000000)
        return evidence

    def test_configured_geometry_dimensions_and_source_hashes_are_bound(self):
        self.assertEqual((self.contract.width,self.contract.height),(640,480))
        for actual,expected in zip(self.contract.mount.center,(.28,0.,.34)):
            self.assertAlmostEqual(actual,expected)
        self.contract.verify_current()
        file=self.root/'config/bridge.yaml';file.write_bytes(file.read_bytes()+b'\n')
        with self.assertRaisesRegex(ValueError,'camera_source_changed'):self.contract.verify_current()

    def test_mismatched_urdf_sensor_mount_or_oversized_source_is_refused(self):
        urdf=self.root/'config/robot.urdf';before=urdf.read_bytes()
        urdf.write_bytes(before.replace(b'.28 0 .12',b'.38 0 .12'))
        with self.assertRaisesRegex(ValueError,'mount_source'):CameraContract.load(self.registry)
        urdf.write_bytes(before)
        self.world_path.write_bytes(self.world_path.read_bytes().replace(b'<width>640</width>',b'<width>999999</width>'))
        with self.assertRaisesRegex(ValueError,'unsupported_camera_source'):
            CameraContract.load(MachineRegistry.load(self.root))

    def test_paired_stamps_and_post_settling_admission_are_required(self):
        evidence=CameraEvidence(self.contract);evidence.tick(10.,10**9)
        image,info=messages();info.header.stamp.nanosec+=1
        evidence.image(image,10.5,1500000001);evidence.info(info,10.5,1500000001)
        with self.assertRaises(InspectionPending):evidence.select(1500000001,1400000000,10.51,1510000000)
        evidence=self.ready();evidence.selected=None
        with self.assertRaises(InspectionPending):evidence.select(1550000000,1500000001,10.55,1550000000)

    def test_image_skew_and_freshness_boundaries_are_inclusive(self):
        evidence=self.ready();evidence.selected=None
        self.assertEqual(evidence.select(1600000000,1400000000,10.6,1600000000),1500000000)
        evidence.selected=None
        with self.assertRaises(InspectionPending):evidence.select(1600000001,1400000000,10.6,1600000001)
        evidence=self.ready();evidence.check(10.7,1700000000)
        with self.assertRaisesRegex(ValueError,'expired'):evidence.check(10.7,1700000001)

    def test_new_frames_do_not_replace_a_selected_frame_or_mutate_its_bytes(self):
        evidence=self.ready();selected=evidence.selected
        image,info=messages(1600000000)
        evidence.image(image,10.6,1600000000);evidence.info(info,10.6,1600000000)
        image.data[0]=200;info.k[0]=0
        self.assertEqual(evidence.select(1550000000,1400000000,10.61,1610000000),1500000000)
        self.assertEqual(evidence.selected,selected)
        self.assertEqual(evidence.images[-1][2][0],7)
        self.assertEqual(len(evidence.images),2)

    def test_oversized_transport_data_is_rejected_before_another_payload_copy(self):
        evidence=CameraEvidence(self.contract);image,_=messages();image.data=bytearray(MAX_IMAGE_BYTES*8)
        tracemalloc.start()
        try:
            before=tracemalloc.get_traced_memory()[0];tracemalloc.reset_peak()
            self.assertFalse(evidence.image(image,10.5,1500000000))
            extra=tracemalloc.get_traced_memory()[1]-before
        finally:tracemalloc.stop()
        self.assertLess(extra,65536)

    def test_replay_future_pre_admission_and_clock_regression_remain_fatal(self):
        for kind in ('replay','future','old','clock'):
            evidence=self.ready();image,info=messages()
            if kind=='future':image.header.stamp.sec+=1
            elif kind=='old':image.header.stamp.sec=0
            if kind=='clock':
                with self.assertRaises(ValueError):evidence.tick(10.6,1400000000)
            else:self.assertFalse(evidence.image(image,10.6,1600000000))
            with self.assertRaises(ValueError):evidence.check(10.61,1610000000)

    def test_finite_calibration_change_is_rejected_without_erasing_prior_failure(self):
        evidence=self.ready();image,info=messages(1600000000)
        info.k[2]+=.5;info.p[2]+=.5
        self.assertFalse(evidence.info(info,10.6,1600000000))
        _,valid=messages(1650000000)
        self.assertFalse(evidence.info(valid,10.65,1650000000))
        with self.assertRaises(ValueError):evidence.check(10.66,1660000000)

    def test_uint_layout_and_calibration_types_never_coerce_invalid_values(self):
        for kind in ('bool_width','bool_step','bigendian','nonbyte','roi_bool','binning_bool','d_large','r_wrong','p_wrong'):
            evidence=CameraEvidence(self.contract);image,info=messages()
            if kind=='bool_width':image.width=True
            elif kind=='bool_step':image.step=True
            elif kind=='bigendian':image.is_bigendian=1
            elif kind=='nonbyte':image.data=[7]*921600
            elif kind=='roi_bool':info.roi.x_offset=False
            elif kind=='binning_bool':info.binning_x=True
            elif kind=='d_large':info.d=[0.]*100000
            elif kind=='r_wrong':info.r[0]=0.
            else:info.p[0]=0.
            accepted=evidence.image(image,10.5,1500000000) and evidence.info(info,10.5,1500000000)
            self.assertFalse(accepted,kind)
