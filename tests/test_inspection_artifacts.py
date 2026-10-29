import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from orbital.inspection_artifacts import CaptureStore,MAX_CAPTURES,read_capture


class InspectionArtifactsTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)
        self.capture_id='a'*32

    def record(self):
        return {'schema_version':2,'valid':True,'observation_id':self.capture_id,
                'camera':{'width':2,'height':1}}

    def save(self):
        store=CaptureStore(self.root,self.capture_id)
        try:return store.save(self.record(),b'\x01\x02\x03'*2,lambda:None)
        except BaseException:
            store.reject();raise
        finally:store.close()

    def test_bundle_roundtrip_preserves_exact_bytes_and_excludes_self_hash(self):
        result=self.save()
        record,image=read_capture(self.root,self.capture_id,result['capture_artifact']['manifest_sha256'])
        self.assertEqual(image,b'P6\n2 1\n255\n'+b'\x01\x02\x03'*2)
        self.assertNotIn('manifest_sha256',record['capture_artifact'])
        self.assertFalse((self.root/result['capture_artifact']['directory']/'pending.json').exists())

    def test_manifest_and_image_tampering_are_detected_on_later_read(self):
        result=self.save();directory=self.root/result['capture_artifact']['directory']
        for name in ('observation.json','image.ppm'):
            file=directory/name;before=file.read_bytes()
            try:
                file.write_bytes(before[:-1]+b'X')
                with self.subTest(name=name),self.assertRaises(ValueError):
                    read_capture(self.root,self.capture_id,result['capture_artifact']['manifest_sha256'])
            finally:file.write_bytes(before)

    def test_existing_capture_cannot_be_overwritten(self):
        result=self.save();file=self.root/result['capture_artifact']['directory']/'observation.json';before=file.read_bytes()
        with self.assertRaises(FileExistsError):self.save()
        self.assertEqual(file.read_bytes(),before)

    def test_symlinked_ancestors_never_create_outside_artifacts(self):
        outside=self.root/'outside';outside.mkdir()
        runtime=self.root/'runtime';runtime.symlink_to(outside,target_is_directory=True)
        with self.assertRaises(OSError):self.save()
        self.assertEqual(list(outside.iterdir()),[])
        runtime.unlink();runtime.mkdir()
        (runtime/'inspection').symlink_to(outside,target_is_directory=True)
        with self.assertRaises(OSError):self.save()
        self.assertEqual(list(outside.iterdir()),[])

    def test_image_symlink_and_hardlinked_or_nonregular_lock_are_refused(self):
        outside=self.root/'outside';outside.write_text('preserve')
        store=CaptureStore(self.root,self.capture_id)
        try:
            (self.root/'runtime/inspection'/self.capture_id/'image.ppm').symlink_to(outside)
            with self.assertRaises(FileExistsError):store.save(self.record(),b'\0'*6,lambda:None)
        finally:store.close()
        self.assertEqual(outside.read_text(),'preserve')
        lock=self.root/'runtime/inspection/.lock';lock.unlink();os.link(outside,lock)
        with self.assertRaisesRegex(ValueError,'invalid_capture_lock'):CaptureStore(self.root,'b'*32)
        lock.unlink();os.mkfifo(lock)
        with self.assertRaisesRegex(ValueError,'invalid_capture_lock'):CaptureStore(self.root,'b'*32)

    def test_capture_count_and_writer_lock_bound_concurrent_storage(self):
        store=CaptureStore(self.root,self.capture_id)
        try:
            with self.assertRaises(BlockingIOError):CaptureStore(self.root,'b'*32)
        finally:store.close()
        parent=self.root/'runtime/inspection'
        for index in range(MAX_CAPTURES-1):(parent/f'{index:032x}').mkdir()
        with self.assertRaisesRegex(ValueError,'capture_store_full'):CaptureStore(self.root,'c'*32)
        self.assertEqual(len(list(parent.iterdir())),MAX_CAPTURES+1)

    def test_interrupt_immediately_after_link_cannot_leave_an_accepted_manifest(self):
        original=os.link
        def interrupted(*args,**kwargs):original(*args,**kwargs);raise KeyboardInterrupt
        with patch('orbital.inspection_artifacts.os.link',interrupted),self.assertRaises(KeyboardInterrupt):self.save()
        self.assertFalse((self.root/'runtime/inspection'/self.capture_id/'observation.json').exists())

    def test_reader_rejects_untrusted_identifiers_paths_and_special_files(self):
        result=self.save();reference=result['capture_artifact']['manifest_sha256']
        for name in ('../escape','b'*33,'A'*32):
            with self.assertRaises(ValueError):read_capture(self.root,name,reference)
        directory=self.root/'runtime/inspection'/self.capture_id
        manifest=directory/'observation.json';value=json.loads(manifest.read_bytes())
        value['camera']['image']['file']='../../escape'
        data=json.dumps(value).encode();manifest.write_bytes(data)
        with self.assertRaisesRegex(ValueError,'image_name'):
            read_capture(self.root,self.capture_id,hashlib.sha256(data).hexdigest())
        manifest.unlink();os.mkfifo(manifest)
        with self.assertRaisesRegex(ValueError,'invalid_capture_file'):
            read_capture(self.root,self.capture_id,reference)

    def test_invalid_or_oversized_payload_never_publishes_a_manifest(self):
        store=CaptureStore(self.root,self.capture_id)
        try:
            with self.assertRaisesRegex(ValueError,'pixels_invalid'):store.save(self.record(),b'\0'*7,lambda:None)
            record=self.record();record['unbounded']='x'*65536
            with self.assertRaisesRegex(ValueError,'manifest_oversized'):store.save(record,b'\0'*6,lambda:None)
        finally:store.close()
        self.assertFalse((self.root/'runtime/inspection'/self.capture_id/'observation.json').exists())
