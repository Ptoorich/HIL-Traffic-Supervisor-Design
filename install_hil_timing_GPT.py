"""Install the four HIL timing source updates; preserves backups and checks versions.

Save this script in the project root and run: python install_hil_timing_GPT.py
The payload contains text edits, not executable encoded commands.
"""
import base64, hashlib, json, os, tempfile, zlib
from datetime import datetime, timezone
from pathlib import Path

DATA = """
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
"""

def sha(text):
    return hashlib.sha256(text.encode('utf-8')).hexdigest()

def main():
    root = Path(__file__).resolve().parent
    if not (root/'supervisor').is_dir():
        raise SystemExit('Save this script in the HIL project root first.')
    specs = json.loads(zlib.decompress(base64.b64decode(DATA)))
    pending = []
    for rel, item in specs.items():
        path = root/rel
        if not path.resolve().is_relative_to(root):
            raise SystemExit('Refusing a path outside the project: '+rel)
        original = path.read_bytes() if path.exists() else None
        text = original.decode('utf-8-sig').replace('\r\n','\n') if original is not None else ''
        if original is not None and sha(text)==item['after']:
            continue
        if (original is None) != (item['before'] is None) or (original is not None and sha(text)!=item['before']):
            raise SystemExit('Version differs from the uploaded source; no files changed: '+rel)
        if 'text' in item:
            updated = item['text']
        else:
            lines = text.splitlines(keepends=True)
            for start, end, replacement in reversed(item['hunks']):
                lines[start:end] = replacement.splitlines(keepends=True)
            updated = ''.join(lines)
        if sha(updated)!=item['after']:
            raise SystemExit('Update checksum mismatch; no files changed: '+rel)
        compile(updated,rel,'exec')
        pending.append((rel,path,original,updated.encode('utf-8')))
    if not pending:
        print('HIL timing update is already installed.'); return
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    backup = root/'supervisor_logs'/'hil_timing_backups_GPT'/stamp
    backup.mkdir(parents=True,exist_ok=False)
    for rel,path,original,updated in pending:
        if (path.read_bytes() if path.exists() else None)!=original:
            raise SystemExit('Source changed during preparation; no code changed: '+rel)
        if original is not None:
            dest = backup/rel; dest.parent.mkdir(parents=True,exist_ok=True); dest.write_bytes(original)
    written = []
    try:
        for rel,path,original,updated in pending:
            with tempfile.NamedTemporaryFile(dir=path.parent,prefix='.hil-timing-',delete=False) as temp:
                temp.write(updated); temp_path = Path(temp.name)
            try:
                os.replace(temp_path,path)
            finally:
                temp_path.unlink(missing_ok=True)
            written.append((path,original))
    except BaseException:
        for path,original in reversed(written):
            if original is None: path.unlink(missing_ok=True)
            else: path.write_bytes(original)
        raise
    print('Updated',len(pending),'files. Backups:',backup)
    print('New HIL runs will write timing_GPT.jsonl and timing_summary_GPT.json.')

if __name__=='__main__':
    main()
