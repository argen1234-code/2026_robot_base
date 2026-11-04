import ast
import hashlib
import json
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[3]
BASE='4014edb2e63edcddbc20475b6d9c74c3eb2e1aab'
CONSOLE='上位机/RobStride_Console/'
ALLOWED={'README.md',CONSOLE+'README.md','上位机/RobStrideConsole.exe',
         CONSOLE+'firmware_profile.json',CONSOLE+'server.py',CONSOLE+'web/app.js',
         CONSOLE+'web/index.html','上位机/双击启动说明.txt'}

def git(*args):
    return subprocess.check_output(['git',*args],cwd=ROOT)

tree={}
for row in git('ls-tree','-rz',BASE).split(b'\0'):
    if not row:continue
    meta,path=row.split(b'\t',1)
    mode,kind,oid=meta.split()
    assert mode!=b'160000'
    if kind==b'blob':tree[path.decode('utf-8')]=oid.decode()
assert all((ROOT/p).is_file() for p in tree),'Original file removed'
hashes=subprocess.check_output(['git','hash-object','--stdin-paths'],cwd=ROOT,
    input=('\n'.join(tree)+'\n').encode('utf-8')).decode().splitlines()
changed=[p for (p,before),after in zip(tree.items(),hashes) if before!=after]
assert set(changed)<=ALLOWED,changed
firmware=[p for p in tree if p.startswith('RobStride05_H743/')]
assert all(p not in changed for p in firmware)

before=git('show',BASE+':'+CONSOLE+'server.py').decode('utf-8')
after=(ROOT/CONSOLE/'server.py').read_text('utf-8')
def ast_methods(source):
    return {node.name:ast.dump(node,include_attributes=False) for node in ast.walk(ast.parse(source))
            if isinstance(node,(ast.FunctionDef,ast.AsyncFunctionDef))}
original_methods=ast_methods(before);new_methods=ast_methods(after)
different=[name for name in original_methods if original_methods[name]!=new_methods[name]]
assert different==['do_GET'],different
extra_assets={'/template-ui.js','/template.css','/signal-theme.css','/distortion-grid.js',
              '/footer-distortion.js','/signal-distortion.js','/pixel-field.js'}
after_ast=ast.parse(after)
for node in ast.walk(after_ast):
    if isinstance(node,ast.Dict):
        keep=[(key,value) for key,value in zip(node.keys,node.values)
              if not isinstance(key,ast.Constant) or key.value not in extra_assets]
        node.keys=[key for key,value in keep];node.values=[value for key,value in keep]
assert ast.dump(after_ast)==ast.dump(ast.parse(before))
old=json.loads(git('show',BASE+':'+CONSOLE+'firmware_profile.json'))
new=json.loads((ROOT/CONSOLE/'firmware_profile.json').read_text('utf-8'))
old.pop('source');new.pop('source')
assert old==new,'Firmware ABI or Flash identity changed'
binary=ROOT/'上位机/RobStrideConsole.exe'
effects_baseline='3af57daa8195a49abe5fc9be86aa44882dd0d027'
business_files=[CONSOLE+name for name in ('web/app.js','web/led.js','bridge.py','profile.py')]
for path in business_files:
    assert git('rev-parse',effects_baseline+':'+path).strip()==git('hash-object',path).strip(),path
report={'passed':True,'baseline':BASE,'baseline_file_count':len(tree),
        'original_files_retained':len(tree),'unchanged_original_git_blobs':len(tree)-len(changed),
        'firmware_files_with_identical_git_blobs':len(firmware),'firmware_changed':False,
        'changed_original_files':changed,'server_changes':'only seven static asset allowlist entries',
        'control_and_validation_methods_unchanged':True,'firmware_profile_only_source_path_changed':True,
        'exe_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),'exe_bytes':binary.stat().st_size,
        'hardware_tested_this_run':False,'effects_baseline':effects_baseline,
        'business_files_identical_to_effects_baseline':business_files}
(ROOT/CONSOLE/'review/preservation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(report,ensure_ascii=False,indent=2))
