"""Verify help-only production edits and actual native / no-KiCad test behavior."""
import argparse
import ast
import json
from pathlib import Path
import shutil
import subprocess
import sys

p=argparse.ArgumentParser()
p.add_argument('--root',type=Path,required=True)
p.add_argument('--out',type=Path,required=True)
a=p.parse_args()
root,out=a.root.resolve(),a.out.resolve()
out.mkdir(parents=True,exist_ok=True)
old=subprocess.check_output(['git','show','bc8071b9ab586dd78629cdf511e4d527b4af33a1:py_tools/check_floorplan.py'],cwd=root,text=True,encoding='utf8')
new=(root/'py_tools/check_floorplan.py').read_text(encoding='utf8')
class WithoutHelp(ast.NodeTransformer):
    def visit_keyword(self,node):
        node=self.generic_visit(node)
        if node.arg=='help':node.value=ast.Constant(value='<help>')
        return node
assert ast.dump(WithoutHelp().visit(ast.parse(old)))==ast.dump(WithoutHelp().visit(ast.parse(new)))
rows=[]
def run(name,argv,expected):
    r=subprocess.run(argv,cwd=root,text=True,encoding='utf8',capture_output=True,timeout=180)
    (out/(name+'.log')).write_text(r.stdout+r.stderr,encoding='utf8')
    assert r.returncode==expected,(name,r.returncode,r.stdout,r.stderr)
    assert 'Traceback' not in r.stdout+r.stderr,(name,r.stderr)
    rows.append({'name':name,'argv':argv,'cwd':str(root),'exit':r.returncode,'log':name+'.log'})
    return r
native=[sys.executable,'-X','utf8']
run('native-publication',native+[str(root/'tests/test_961_connector_publication.py')],0)
system=shutil.which('python3')
probe=run('system-pcbnew-probe',[system,'-c','import importlib.util,sys; print(sys.version); assert importlib.util.find_spec("pcbnew") is None'],0)
run('no-native-publication',[system,'-X','utf8',str(root/'tests/test_961_connector_publication.py')],77)
help=run('floorplan-help',native+[str(root/'py_tools/check_floorplan.py'),'--help'],0)
words=' '.join(help.stdout.split())
assert 'independent copper and geometry requirements can still fail' in words
assert 'class-only entries impose no edge, seating or maximum overhang requirement' in words
fixtures=root/'docs/issue-961-evidence/reproduction/candidate-bc8071'
run('floorplan-existing-control',native+[str(root/'py_tools/check_floorplan.py'),
    str(fixtures/'original/esp_prog.kicad_pcb'),'--intent',str(fixtures/'zero.intent.json'),
    '--clearance','.25','--board-edge-clearance','.25','--json',str(out/'grade.json')],0)
before=json.loads((fixtures/'original/zero-c0.25-e0.25.cli.json').read_text(encoding='utf8'))
after=json.loads((out/'grade.json').read_text(encoding='utf8'))
assert before==after
(out/'results.json').write_text(json.dumps({'revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
    'production_AST_identical_except_help_strings':True,'actual_grade_JSON_unchanged':True,
    'no_native_is_skip_not_pass':True,'runs':rows},indent=2)+'\n',encoding='utf8')
print('Help AST, unchanged actual grade, native pass and absent-native skip77 verified')
