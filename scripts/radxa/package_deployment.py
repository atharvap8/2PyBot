"""Create a commit-bound firmware/Android bundle after CI builds and checks succeed."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile

ROOT=Path(__file__).resolve().parents[2]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--firmware',type=Path,default=ROOT/'build/BaseLink')
    parser.add_argument('--apk',type=Path,default=ROOT/'software/android/app/build/outputs/apk/debug/app-debug.apk')
    parser.add_argument('--output',type=Path,default=ROOT/'build/deployment')
    args=parser.parse_args()
    commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    firmware=json.loads((args.firmware/'manifest.json').read_text())
    metadata=json.loads((args.apk.parent/'output-metadata.json').read_text())['elements'][0]
    version=(ROOT/'VERSION').read_text().strip()
    assert firmware['source_commit']==commit and not firmware['source_dirty'] and firmware.get('source_fingerprint')
    assert metadata['versionName']==version and firmware['version']==version
    directory=args.output/'contents';directory.mkdir(parents=True,exist_ok=True)
    (directory/'firmware').mkdir(exist_ok=True);(directory/'android').mkdir(exist_ok=True)
    image=args.firmware/'BaseLink.ino.bin'
    assert hashlib.sha256(image.read_bytes()).hexdigest()==firmware['sha256']
    shutil.copy2(image,directory/'firmware/application.bin')
    shutil.copy2(args.firmware/'manifest.json',directory/'firmware/manifest.json')
    shutil.copy2(args.apk,directory/'android/app.apk')
    android={'version':version,'version_code':metadata['versionCode'],'source_commit':commit,
        'bytes':args.apk.stat().st_size,'sha256':hashlib.sha256(args.apk.read_bytes()).hexdigest()}
    (directory/'android/manifest.json').write_text(json.dumps(android,indent=2)+'\n')
    files=['firmware/application.bin','firmware/manifest.json','android/app.apk','android/manifest.json']
    bundle={'schema':1,'source_commit':commit,'version':version,'sha256':{
        name:hashlib.sha256((directory/name).read_bytes()).hexdigest() for name in files}}
    (directory/'bundle.json').write_text(json.dumps(bundle,indent=2)+'\n')
    archive=args.output/f'2pybot-deploy-{commit}.tar.gz'
    with tarfile.open(archive,'w:gz') as file:
        for name in files+['bundle.json']:file.add(directory/name,arcname=name)
    print(archive)


if __name__=='__main__':main()
