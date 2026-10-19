"""Check VERSION or print its next revision; never edits build/history files."""
import argparse
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"software/radxa/console"))
from console.versioning import next_revision,validate_version

if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--next",action="store_true")
    parser.add_argument("--check",action="store_true")
    args=parser.parse_args();version=(ROOT/"VERSION").read_text().strip()
    validate_version(version)
    print(next_revision(version) if args.next else "Version policy OK: "+version)
