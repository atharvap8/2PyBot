"""Register a locally compiled/published build on the bot; no firmware flashing."""
import argparse
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"software/radxa/console"))
from console.firmware import ROOT as STATE_ROOT, register_build

if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("incoming",type=Path)
    parser.add_argument("--installed-candidate",action="store_true")
    parser.add_argument("--state-dir",type=Path,default=STATE_ROOT)
    args=parser.parse_args()
    result=register_build(args.state_dir,args.incoming,args.installed_candidate)
    print("Registered",result["version"],result["build_id"],"as", "rollback candidate" if args.installed_candidate else "latest compiled build")
