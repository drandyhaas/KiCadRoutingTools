"""Execute all four committed #961 tests on preserved CRLF and LF-only source copies."""
import argparse
import contextlib
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys
import types
import unittest

p=argparse.ArgumentParser()
p.add_argument('--root',type=Path,required=True)
p.add_argument('--revision',required=True)
p.add_argument('--out',type=Path,required=True)
a=p.parse_args()
root,out=a.root.resolve(),a.out.resolve()
out.mkdir(parents=True,exist_ok=True)
def git(*args):return subprocess.check_output(['git',*args],cwd=root)
def text(*args):return git(*args).decode('utf8').strip()
sha=lambda b:hashlib.sha256(b).hexdigest()
current=text('rev-parse','HEAD')
# Execute the exact target test object against independently identical runtime
# trees; preserve the existing reviewer checkout/evidence without overwriting it.
roots=text('ls-tree','--name-only',current).splitlines()
identities={}
for path in roots:
    if path in ('docs','tests'):continue
    old,new=text('rev-parse',current+':'+path),text('rev-parse',a.revision+':'+path)
    assert old==new,path
    identities[path]={'reviewer_checkout':old,'target_revision':new}
testpath='tests/test_961_connector_geometry.py'
code=git('show',a.revision+':'+testpath)
(out/'executed_test_source.py').write_bytes(code)
source=root/'kicad_files/esp_prog.kicad_pcb'
raw=source.read_bytes()
canonical=raw.replace(b'\r\n',b'\n')
assert sha(raw)=='165302e6a4f7aacdd64b3174df27ed8ddb19fd5f1f7effadd8d92205e25a120e'
assert sha(canonical)=='a9945bb0940f79672b7c6e32b7a6b9d0b135bf78030e19fcdb88e65c2139903f'
assert canonical==git('show',a.revision+':kicad_files/esp_prog.kicad_pcb')
lf=out/'lf-source.kicad_pcb'
lf.write_bytes(canonical)
assert b'\r' not in lf.read_bytes()
# Preserve every existing source sibling; source currently has none.
sys.path[:0]=[str(root/x) for x in ('py_router','py_placer','py_tools')]
from copy_board import SIBLING_EXTS
import shutil
for ext in SIBLING_EXTS:
    sibling=source.with_suffix(ext)
    if sibling.exists():shutil.copy2(sibling,lf.with_suffix(ext))
rows=[]
for name,board in [('normal',source),('lf',lf)]:
    module=types.ModuleType('independent_961_'+name)
    module.__file__=str(root/testpath)
    exec(compile(code,str(root/testpath),'exec'),module.__dict__)
    module.SOURCE=board
    suite=unittest.defaultTestLoader.loadTestsFromModule(module)
    assert suite.countTestCases()==4
    stream=io.StringIO()
    with contextlib.redirect_stdout(stream),contextlib.redirect_stderr(stream):
        result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    (out/(name+'.log')).write_text(stream.getvalue(),encoding='utf8')
    row={'case':name,'SOURCE':str(board),'raw_sha256':sha(board.read_bytes()),
        'canonical_sha256':sha(board.read_bytes().replace(b'\r\n',b'\n')),
        'testsRun':result.testsRun,'failures':len(result.failures),'errors':len(result.errors),
        'skipped':len(result.skipped),'pass':result.wasSuccessful(),'log':name+'.log'}
    rows.append(row)
    assert result.wasSuccessful(),stream.getvalue()
assert source.read_bytes()==raw
assert lf.read_bytes()==canonical
(out/'results.json').write_text(json.dumps({'revision':a.revision,'reviewer_checkout':current,
    'execution':'exact target Git test object, repository __file__, explicit per-arm SOURCE override',
    'test_git_blob_sha256':sha(code),'original_source_unchanged':True,
    'source_git_blob_equals_LF_normalized_working_file':True,
    'unchanged_production_and_original_fixture_identities':identities,
    'command':[sys.executable,'-X','utf8',str(Path(__file__).resolve()),'--root',str(root),
               '--revision',a.revision,'--out',str(out)],'rows':rows},indent=2)+'\n',encoding='utf8')
print('8/8 tests pass: four with original CRLF source, four with only LF normalization')
