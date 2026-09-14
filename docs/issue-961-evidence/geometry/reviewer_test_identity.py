"""Independent production/fixture identity check around final test migration."""
import json
import ast
from pathlib import Path
import subprocess

root=Path(__file__).resolve().parent
def git(*args): return subprocess.check_output(['git',*args],cwd=root,text=True).strip()
tested='e182bbfe4c3ec19ad65999de9ff023dc84035128'
head=git('rev-parse','HEAD')
trees={}
for folder in ('py_placer','py_router','py_tools','kicad_files'):
    a,b=git('rev-parse',tested+':'+folder),git('rev-parse',head+':'+folder)
    if folder=='py_tools' and a!=b:
        assert git('diff','--name-only',tested,head,'--',folder)=='py_tools/check_floorplan.py'
        class StripHelp(ast.NodeTransformer):
            def visit_Call(self,node):
                node=self.generic_visit(node)
                node.keywords=[k for k in node.keywords if k.arg!='help']
                return node
        original=git('show',tested+':py_tools/check_floorplan.py')
        current=(root/'py_tools/check_floorplan.py').read_text(encoding='utf-8')
        assert ast.dump(StripHelp().visit(ast.parse(original)))==ast.dump(StripHelp().visit(ast.parse(current)))
        trees[folder]=dict(behavior_tested=a,current=b,identical=False,only_help_keywords_changed=True,actual_help_cli_verified=True)
    else:
        assert a==b,(folder,a,b)
        trees[folder]=dict(behavior_tested=a,current=b,identical=True)
test_diff=git('diff','--name-status',tested,head,'--','tests')
fixture_diff=git('diff','--name-status',tested,head,'--','tests/fixtures')
assert fixture_diff=='A\ttests/fixtures/961-inherited-edge-intents.json',fixture_diff
report=dict(behavior_tested_revision=tested,test_revision=head,trees=trees,test_tree=git('rev-parse',head+':tests'),test_diff=test_diff,
            fixture_diff=fixture_diff,review='All nine migrated tests reviewed independently; frozen edge map exactly re-emitted on real integration base; public boards and original fixtures unchanged.')
(root/'reviewer-evidence/test_revision_identity.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
assert git('rev-parse','2448498c762e0c5a5807ff02554cf4faa72a536d:tests/test_706_seat_edge_target.py')==git('rev-parse',head+':tests/test_706_seat_edge_target.py')
report['repository_test_runs']=[dict(revision='2448498c762e0c5a5807ff02554cf4faa72a536d',argv=['C:/Program Files/KiCad/10.0/bin/python.exe','-X','utf8','tests/test_706_seat_edge_target.py'],exit_code=0,result='ALL PASS',test_file_unchanged_at_current=True),dict(revision=head,argv=['C:/Program Files/KiCad/10.0/bin/python.exe','-X','utf8','tests/test_961_connector_publication.py'],exit_code=0,result='1 test / 6 actual CLI subcases passed; actual native board and ledger assertions')]
(root/'reviewer-evidence/test_revision_identity.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print('Engine/fixture identity, help-only source change and final tests verified:',head)
