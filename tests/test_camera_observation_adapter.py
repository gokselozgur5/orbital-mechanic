import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import unittest
from unittest.mock import patch

from orbital import ros_proximity
from orbital.inspection_artifacts import CaptureStore,read_capture
from proximity_fixture import Harness
import test_map_binding as map_fixture

ROOT = Path(__file__).resolve().parents[1]


class CameraObservationAdapterTests(unittest.TestCase):
    def setUp(self):
        map_fixture.MapBindingTests.setUp(self)
        for name in ('config/robot.urdf','config/bridge.yaml'):
            shutil.copyfile(ROOT/name,self.root/name)

    def read(self,harness,machine='pump',timeout=3.):
        return harness.run(invoke=lambda:ros_proximity.capture_machine_observation(
            self.root,machine,timeout=timeout,now=lambda:harness.wall))

    def accepted_files(self):
        return list((self.root/'runtime').glob('inspection/*/observation.json'))

    def test_each_machine_captures_real_bytes_pose_calibration_and_integrity_reference(self):
        for machine,y in (('pump',1.8),('cooler',0.),('power_unit',-1.8)):
            harness=Harness(self);harness.pose=(2.,y,0.)
            result=self.read(harness,machine);camera=result['camera']
            self.assertEqual(result['schema_version'],2)
            self.assertEqual(camera['stamp_ns'],camera['robot_state']['pose_stamp_ns'])
            self.assertIn(('camera_optical_frame',camera['stamp_ns']),harness.camera_lookups)
            for actual,expected in zip(camera['camera_pose_xyz_xyzw'][:3],[2.28,y,.34]):
                self.assertAlmostEqual(actual,expected)
            self.assertEqual(camera['telemetry_offset_ns'],camera['stamp_ns']-result['sample_stamp_ns'])
            record,image=read_capture(self.root,result['observation_id'],result['capture_artifact']['manifest_sha256'])
            self.assertEqual(record['machine_id'],machine)
            self.assertEqual(image,b'P6\n640 480\n255\n'+bytes((7,31,193))*(640*480))
            self.assertEqual(camera['source_sha256'].keys(),{'config/robot.urdf','config/bridge.yaml'})
            self.assertNotIn('fault',json.dumps(record))
            self.assertTrue(harness.destroyed and harness.unregistered and harness.shutdown)

    def test_image_and_telemetry_times_are_preserved_in_both_orderings(self):
        for first,offset in ((1540000000,-60000000),(1660000000,60000000)):
            harness=Harness(self);harness.next_camera=first;harness.camera_order=('info','image')
            result=self.read(harness)
            self.assertEqual(result['camera']['telemetry_offset_ns'],offset)
            self.assertIn(('base_footprint',result['camera']['stamp_ns']),harness.camera_lookups)
            self.assertEqual(result['robot_state']['pose_stamp_ns'],result['sample_stamp_ns'])

    def test_missing_image_info_or_publisher_cannot_publish_a_capture(self):
        for attribute in ('drop_image','drop_info','missing_camera_publisher'):
            harness=Harness(self);setattr(harness,attribute,True)
            with self.subTest(attribute=attribute),self.assertRaises(ValueError):self.read(harness,timeout=2.)
            self.assertEqual(self.accepted_files(),[])
            self.assertTrue(harness.destroyed and harness.unregistered and harness.shutdown)

    def test_invalid_image_or_camera_info_is_sticky_even_before_selection(self):
        for kind in ('size','encoding','step','frame','future','info_stamp','calibration','roi','intrinsics'):
            harness=Harness(self)
            def damage(topic,msg):
                if topic=='image':
                    if kind=='size':msg.data=msg.data[:-1]
                    elif kind=='encoding':msg.encoding='bgr8'
                    elif kind=='step':msg.step+=1
                    elif kind=='frame':msg.header.frame_id='base_link'
                    elif kind=='future':msg.header.stamp.sec+=1
                else:
                    if kind=='info_stamp':msg.header.stamp.nanosec+=1
                    elif kind=='calibration':msg.k[0]=float('nan')
                    elif kind=='roi':msg.roi.x_offset=1
                    elif kind=='intrinsics':msg.k[0]*=2
            harness.camera_hook=damage
            with self.subTest(kind=kind),self.assertRaises(ValueError):self.read(harness)
            self.assertEqual(self.accepted_files(),[])

    def test_camera_transform_must_match_the_configured_mount(self):
        harness=Harness(self);harness.wrong_camera_tf=True
        with self.assertRaisesRegex(ValueError,'camera_transform_mount'):self.read(harness)
        self.assertEqual(self.accepted_files(),[])

    def test_missing_exact_camera_transform_has_no_latest_or_new_frame_fallback(self):
        harness=Harness(self)
        original=harness.modules['tf2_ros'].Buffer.lookup_transform
        def lookup(buffer,parent,child,stamp):
            if child=='camera_optical_frame':raise LookupError('no camera transform')
            return original(buffer,parent,child,stamp)
        with patch.object(harness.modules['tf2_ros'].Buffer,'lookup_transform',lookup),self.assertRaises(ValueError):
            self.read(harness)
        self.assertEqual(self.accepted_files(),[])

    def test_motion_expiry_or_source_change_during_storage_rejects_the_bundle(self):
        write=CaptureStore.write
        urdf=self.root/'config/robot.urdf';before=urdf.read_bytes()
        for kind in ('motion','expiry','source','publisher','mode','clock'):
            harness=Harness(self)
            def delayed(store,name,chunks):
                write(store,name,chunks)
                if name=='pending.json':
                    if kind=='motion':harness.moving=True
                    elif kind=='expiry':harness.wall+=.3
                    elif kind=='source':urdf.write_bytes(before+b'\n')
                    elif kind=='publisher':harness.changed_gid=True
                    elif kind=='mode':harness.clock_mode_changed=True
                    else:harness.sim-=100000000
            try:
                with patch.object(CaptureStore,'write',delayed),self.subTest(kind=kind),self.assertRaises(ValueError):self.read(harness)
                self.assertEqual(self.accepted_files(),[])
                self.assertTrue(harness.destroyed and harness.unregistered and harness.shutdown)
            finally:urdf.write_bytes(before)

    def test_failure_after_manifest_publication_removes_the_acceptance_marker(self):
        harness=Harness(self);original=CaptureStore.verify_directory
        def late(store):
            original(store);harness.wall+=.3
        with patch.object(CaptureStore,'verify_directory',late),self.assertRaises(ValueError):self.read(harness)
        self.assertEqual(self.accepted_files(),[])

    def test_manifest_digest_preparation_cannot_bypass_final_freshness(self):
        from orbital import inspection_artifacts
        harness=Harness(self);original=inspection_artifacts.hashlib.sha256
        def digest(data=b'',**kwargs):
            if b'"capture_artifact":' in data:harness.wall+=.3
            return original(data,**kwargs)
        with patch.object(inspection_artifacts.hashlib,'sha256',digest),self.assertRaises(ValueError):self.read(harness)
        self.assertEqual(self.accepted_files(),[])

    def test_image_changed_during_publication_fails_final_integrity_readback(self):
        harness=Harness(self);original=CaptureStore.verify_directory
        def changed(store):
            original(store)
            file=self.root/'runtime/inspection'/store.observation_id/'image.ppm'
            data=file.read_bytes();file.write_bytes(data[:-1]+b'X')
        with patch.object(CaptureStore,'verify_directory',changed),self.assertRaisesRegex(ValueError,'artifact_changed'):
            self.read(harness)
        self.assertEqual(self.accepted_files(),[])

    def test_destination_rename_during_storage_cannot_leave_a_complete_bundle(self):
        harness=Harness(self);original=CaptureStore.write
        def moved(store,name,chunks):
            original(store,name,chunks)
            if name=='pending.json':
                directory=self.root/'runtime/inspection'/store.observation_id
                directory.rename(directory.with_name('moved'))
        with patch.object(CaptureStore,'write',moved),self.assertRaises(OSError):self.read(harness)
        self.assertEqual(self.accepted_files(),[])
        self.assertTrue(harness.destroyed and harness.unregistered and harness.shutdown)

    def test_write_failure_or_interrupt_cleans_ros_and_never_returns_partial_success(self):
        for error in (OSError('controlled disk failure'),KeyboardInterrupt()):
            harness=Harness(self)
            with patch.object(CaptureStore,'write',side_effect=error),self.assertRaises(type(error)):self.read(harness)
            self.assertEqual(self.accepted_files(),[])
            self.assertTrue(harness.destroyed and harness.unregistered and harness.shutdown)

    def test_cli_camera_flag_invokes_the_real_capture_and_emits_only_references(self):
        spec=importlib.util.spec_from_file_location('camera_cli',ROOT/'scripts/machine-observation.py')
        cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)
        harness=Harness(self);output=io.StringIO()
        def capture(root,machine,**kwargs):
            return ros_proximity.capture_machine_observation(root,machine,now=lambda:harness.wall,**kwargs)
        with patch.dict(os.environ,{'ROS_DOMAIN_ID':'42','ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST'}), \
                patch.object(cli,'ROOT',self.root),patch.object(cli,'capture_machine_observation',capture),contextlib.redirect_stdout(output):
            self.assertEqual(harness.run(invoke=lambda:cli.main(['pump','--with-camera','--timeout','3'])),0)
        result=json.loads(output.getvalue())
        self.assertIn('manifest_sha256',result['capture_artifact'])
        self.assertLess(len(output.getvalue()),20000)
        self.assertNotIn('data',result['camera'])

    def test_cli_camera_failures_and_interruption_emit_invalid_results(self):
        spec=importlib.util.spec_from_file_location('camera_cli_failures',ROOT/'scripts/machine-observation.py')
        cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)
        for error,code in ((OSError('controlled disk failure'),2),(KeyboardInterrupt(),130)):
            harness=Harness(self);output=io.StringIO()
            def capture(root,machine,**kwargs):
                return ros_proximity.capture_machine_observation(root,machine,now=lambda:harness.wall,**kwargs)
            with patch.dict(os.environ,{'ROS_DOMAIN_ID':'42','ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST'}), \
                    patch.object(cli,'ROOT',self.root),patch.object(cli,'capture_machine_observation',capture), \
                    patch.object(CaptureStore,'write',side_effect=error),contextlib.redirect_stdout(output):
                self.assertEqual(harness.run(invoke=lambda:cli.main(['pump','--with-camera','--timeout','3'])),code)
            result=json.loads(output.getvalue())
            self.assertFalse(result['valid']);self.assertNotIn('camera',result)
            self.assertTrue(harness.destroyed and harness.unregistered and harness.shutdown)
