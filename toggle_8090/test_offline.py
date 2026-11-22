#!/usr/bin/env python3
"""Offline validation, no actual network, systemd or hardware writes."""
import ast
import contextlib
import io
import pathlib
import runpy
import subprocess
import sys
import tempfile

ROOT=pathlib.Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
import install


def source_variants():
    # Included source baselines are only used in this development environment.
    p=pathlib.Path('/mnt/data/_hud_source')
    return [p/name for name in ('8090_center_v22.py','8090_v22_rightcol_test.py') if (p/name).exists()]


def check_rendered(source):
    install.TARGET=source
    src,patched=install.payload()
    assert src==source.read_bytes()
    with tempfile.TemporaryDirectory() as t:
        run=pathlib.Path(t)/'mock_source.py'
        run.write_text(patched.decode('utf-8').replace(
            "'/home/orangepi/OP5_8090_FUSION_TOGGLE_FINAL_V1'",
            repr(str(ROOT))), encoding='utf-8')
        ns=runpy.run_path(str(run),run_name='__offline__')
        html=ns['HTML']
        for need in ('hud-follow-slot','op5-center-v2-slide',
                     'id="op5-v3-switch"','op5-8090-slide',
                     'await window.OP5FusionMode.arm();',
                     'await window.OP5FusionMode.heartbeat();',
                     'await window.OP5FusionMode.stop();'):
            assert need in html,need
        assert html.count('OP5_8090_FUSION_TOGGLE_FINAL_V1_JS')==1
        assert html.count('id="op5-v3-switch"')==1
        # Strict denial of cross-origin movement commands
        seen=[]
        class Fake:
            path='/op5-v3-toggle/arm'
            headers={'Host':'127.0.0.1:8090','Origin':'http://evil.invalid'}
            def send_error(self,status,*args):seen.append(status)
        ns['_v3_proxy'](Fake(),'POST')
        assert seen==[403],seen
        print('PASS 8090 original HUD preserved + no duplicate UI + cross-origin POST blocked:', source.name)


def simulate_install_rollback(source):
    with tempfile.TemporaryDirectory() as t:
        install.TARGET=pathlib.Path(t)/'dual_overlay_switch.py'
        install.MANIFEST=pathlib.Path(t)/'state'/'manifest.json'
        raw=source.read_bytes()
        install.TARGET.write_bytes(raw)
        prev={name:getattr(install,name) for name in ('call','probe','enabled','active','atomic','verify_running')}
        install.call=lambda *args:'active' if args[:2]==('systemctl','is-active') else 'enabled'
        install.probe=lambda port,path,timeout=3: (
            {'active':False} if port==8093 else
            {'armed':False,'stop_pending':False} if (port==8101 and path=='/api/status') else
            {'version':'V3_STAGE1_FUSED_CAMERA_GUARD','ok':True} if path=='/api/health' else
            {'ready':False, 'reason':'offline test'} )
        install.enabled=lambda s:False
        install.active=lambda s:True
        install.atomic=lambda blob:install.TARGET.write_bytes(blob)
        install.verify_running=lambda:None
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                install.check()
                assert install.TARGET.read_bytes()==raw
                install.apply()
                assert install.MANIFEST.is_file()
                assert b'OP5_8090_FUSION_TOGGLE_FINAL_V1' in install.TARGET.read_bytes()
                install.rollback()
            assert install.TARGET.read_bytes()==raw
            assert not install.MANIFEST.exists()
            print('PASS dry-run unchanged, simulated apply, exact-byte rollback')
        finally:
            for name,obj in prev.items():setattr(install,name,obj)


def main():
    ast.parse((ROOT/'runtime_inject.py').read_text('utf-8'))
    ast.parse((ROOT/'install.py').read_text('utf-8'))
    try:subprocess.run(['node','--check',str(ROOT/'fusion_toggle.js')],check=True,stdout=subprocess.PIPE)
    except FileNotFoundError:print('SKIP JS syntax: Node not available')
    else:print('PASS JS syntax')
    files=source_variants()
    if files:
        for f in files:check_rendered(f)
        simulate_install_rollback(files[0])
    else:print('Source snapshots unavailable: core syntax checked only')
    print('OFFLINE TEST PASS: no motor or radar commands issued')

if __name__=='__main__':main()
