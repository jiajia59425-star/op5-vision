#!/usr/bin/env python3
"""Systemd one-shot failsafe restoration of the original V3 radar config."""
import hashlib
import os
import pathlib
import sys
import tempfile

def sha(data):
    return hashlib.sha256(data).hexdigest()

def atomic_bytes(path, raw):
    stat = path.stat()
    fd, tmp = tempfile.mkstemp(prefix='.v3cal-restore-', dir=str(path.parent))
    try:
        os.fchmod(fd, stat.st_mode & 0o7777)
        os.fchown(fd, stat.st_uid, stat.st_gid)
        with os.fdopen(fd, 'wb') as f:
            f.write(raw); f.flush(); os.fsync(f.fileno())
        os.replace(tmp, path)
        dfd = os.open(str(path.parent), os.O_RDONLY)
        try: os.fsync(dfd)
        finally: os.close(dfd)
    finally:
        if os.path.exists(tmp): os.unlink(tmp)

def restore(cfg,backup,patched_sha):
    raw=backup.read_bytes()
    if not raw: raise RuntimeError('backed up file empty')
    current=cfg.read_bytes()
    if sha(current)==sha(raw):
        print('ALREADY_RESTORED', flush=True)
        return True
    if sha(current)!=patched_sha:
        # Never destroy an independently edited config; require human attention.
        print('MANUAL_INTERVENTION_REQUIRED: config differs from expected test version',file=sys.stderr,flush=True)
        return False
    atomic_bytes(cfg,raw)
    if cfg.read_bytes()!=raw: raise RuntimeError('restore read-back mismatch')
    print('RESTORE_SUCCESS: original config bytes restored',flush=True)
    return True

if __name__=='__main__':
    if len(sys.argv)!=4: raise SystemExit('Usage: restore.py config backup expected_test_sha256')
    ok=restore(pathlib.Path(sys.argv[1]),pathlib.Path(sys.argv[2]),sys.argv[3])
    raise SystemExit(0 if ok else 2)
