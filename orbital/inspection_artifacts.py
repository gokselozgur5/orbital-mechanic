"""Exclusive, bounded local capture bundles; no caller-selected paths or overwrite."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import stat

from orbital.camera_observation import MAX_IMAGE_BYTES, canonical
from orbital.proximity import Unavailable

MAX_CAPTURES = 64
MAX_MANIFEST_BYTES = 65536
DIRECTORY_FLAGS = os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW
FILE_FLAGS = os.O_NOFOLLOW|os.O_NONBLOCK


def identifier(value):
    if not isinstance(value,str) or re.fullmatch('[0-9a-f]{32}',value) is None:
        raise ValueError('invalid_capture_identifier')
    return value


def directory(parent, name, create=False):
    if create:
        try:os.mkdir(name,mode=0o700,dir_fd=parent)
        except FileExistsError:pass
    return os.open(name,DIRECTORY_FLAGS,dir_fd=parent)


def read_file(parent, name, limit):
    fd = os.open(name,os.O_RDONLY|FILE_FLAGS,dir_fd=parent)
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink!=1 or not 0<info.st_size<=limit:
            raise ValueError('invalid_capture_file')
        chunks,size = [],0
        while size<=limit:
            part = os.read(fd,min(65536,limit+1-size))
            if not part:break
            chunks.append(part);size+=len(part)
        if size!=info.st_size or size>limit:
            raise ValueError('capture_file_changed_or_oversized')
        return b''.join(chunks)
    finally:os.close(fd)


class CaptureStore:
    def __init__(self, root, observation_id):
        self.root = Path(root).resolve()
        self.observation_id = identifier(observation_id)
        self.fds = []
        self.bundle = self.lock = None
        self.manifest_identity = None
        try:
            root_fd = os.open(self.root,DIRECTORY_FLAGS);self.fds.append(root_fd)
            runtime = directory(root_fd,'runtime',True);self.fds.append(runtime)
            parent = directory(runtime,'inspection',True);self.fds.append(parent)
            self.lock = os.open('.lock',os.O_RDWR|os.O_CREAT|FILE_FLAGS,0o600,dir_fd=parent)
            self.fds.append(self.lock)
            info = os.fstat(self.lock)
            if not stat.S_ISREG(info.st_mode) or info.st_nlink!=1:
                raise ValueError('invalid_capture_lock')
            fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            with os.scandir(parent) as entries:
                count = 0
                for entry in entries:
                    if entry.name=='.lock':continue
                    count+=1
                    if count>=MAX_CAPTURES:
                        raise Unavailable('capture_store_full')
            os.mkdir(self.observation_id,mode=0o700,dir_fd=parent)
            self.bundle = directory(parent,self.observation_id);self.fds.append(self.bundle)
        except BaseException:
            self.close()
            raise

    def write(self, name, chunks):
        fd = os.open(name,os.O_WRONLY|os.O_CREAT|os.O_EXCL|FILE_FLAGS,0o600,dir_fd=self.bundle)
        try:
            for chunk in chunks:
                view = memoryview(chunk)
                while view:
                    count = os.write(fd,view)
                    if count<=0:raise OSError('capture_write_failed')
                    view = view[count:]
            os.fsync(fd)
        finally:os.close(fd)

    def save(self, record, pixels, guard):
        if record.get('observation_id')!=self.observation_id or record.get('schema_version')!=2 or record.get('valid') is not True:
            raise ValueError('invalid_capture_record')
        camera = record['camera']
        size = camera['width']*camera['height']*3
        if type(pixels) is not bytes or len(pixels)!=size or not 0<size<=MAX_IMAGE_BYTES:
            raise ValueError('capture_pixels_invalid')
        header = ('P6\n%d %d\n255\n'%(camera['width'],camera['height'])).encode()
        image_hash = hashlib.sha256(header);image_hash.update(pixels)
        camera['image'] = {'file':'image.ppm','media_type':'image/x-portable-pixmap',
                           'bytes':len(header)+size,'sha256':image_hash.hexdigest(),
                           'pixels_sha256':hashlib.sha256(pixels).hexdigest()}
        record['capture_artifact'] = {'directory':'runtime/inspection/'+self.observation_id,
                                      'manifest':'observation.json'}
        manifest = (canonical(record)+'\n').encode()
        if len(manifest)>MAX_MANIFEST_BYTES:
            raise ValueError('capture_manifest_oversized')
        manifest_digest = hashlib.sha256(manifest).hexdigest()
        self.write('image.ppm',(header,pixels))
        self.write('pending.json',(manifest,))
        pending = os.stat('pending.json',dir_fd=self.bundle,follow_symlinks=False)
        self.manifest_identity = (pending.st_dev,pending.st_ino)
        if (read_file(self.bundle,'image.ppm',MAX_IMAGE_BYTES+64)!=header+pixels
                or read_file(self.bundle,'pending.json',MAX_MANIFEST_BYTES)!=manifest):
            raise ValueError('capture_write_verification_failed')
        guard()
        # An exclusive run directory never contains an earlier accepted record.
        os.link('pending.json','observation.json',src_dir_fd=self.bundle,dst_dir_fd=self.bundle,follow_symlinks=False)
        os.unlink('pending.json',dir_fd=self.bundle)
        os.fsync(self.bundle)
        self.verify_directory()
        if (read_file(self.bundle,'observation.json',MAX_MANIFEST_BYTES)!=manifest
                or read_file(self.bundle,'image.ppm',MAX_IMAGE_BYTES+64)!=header+pixels):
            raise ValueError('capture_artifact_changed')
        guard()
        record['capture_artifact']['manifest_sha256'] = manifest_digest
        return record

    def verify_directory(self):
        # Compare directory identities without following a newly introduced link.
        fds = []
        try:
            current = os.open(self.root,DIRECTORY_FLAGS);fds.append(current)
            for name in ('runtime','inspection',self.observation_id):
                current = directory(current,name);fds.append(current)
            if (os.fstat(current).st_dev,os.fstat(current).st_ino)!=(os.fstat(self.bundle).st_dev,os.fstat(self.bundle).st_ino):
                raise ValueError('capture_directory_changed')
        finally:
            for fd in reversed(fds):os.close(fd)

    def reject(self):
        try:info = os.stat('observation.json',dir_fd=self.bundle,follow_symlinks=False)
        except FileNotFoundError:return
        if (info.st_dev,info.st_ino)==self.manifest_identity:
            os.unlink('observation.json',dir_fd=self.bundle)

    def close(self):
        for fd in reversed(self.fds):os.close(fd)
        self.fds.clear()


def read_capture(root, observation_id, manifest_sha256):
    """Verify a retained artifact against its returned digest, never restore freshness."""
    identifier(observation_id)
    if not isinstance(manifest_sha256,str) or re.fullmatch('[0-9a-f]{64}',manifest_sha256) is None:
        raise ValueError('invalid_capture_digest')
    fds = []
    try:
        current = os.open(Path(root).resolve(),DIRECTORY_FLAGS);fds.append(current)
        for name in ('runtime','inspection',observation_id):
            current = directory(current,name);fds.append(current)
        raw = read_file(current,'observation.json',MAX_MANIFEST_BYTES)
        if hashlib.sha256(raw).hexdigest()!=manifest_sha256:
            raise ValueError('capture_manifest_digest_mismatch')
        record = json.loads(raw)
        if record.get('observation_id')!=observation_id or record.get('schema_version')!=2 or record.get('valid') is not True:
            raise ValueError('invalid_capture_manifest')
        info = record['camera']['image']
        if info['file']!='image.ppm':raise ValueError('invalid_capture_image_name')
        image = read_file(current,'image.ppm',MAX_IMAGE_BYTES+64)
        if len(image)!=info['bytes'] or hashlib.sha256(image).hexdigest()!=info['sha256']:
            raise ValueError('capture_image_digest_mismatch')
        return record,image
    finally:
        for fd in reversed(fds):os.close(fd)
