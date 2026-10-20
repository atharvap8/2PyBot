"""Digest tracked firmware/toolchain inputs, independently of documentation-only commits."""
import hashlib
from pathlib import Path
import subprocess

INPUTS = ("firmware/BaseLink", "scripts/firmware/build.py", "scripts/firmware/toolchain.json",
          "scripts/firmware/fingerprint.py", "software/radxa/console/console/versioning.py", "VERSION")


def digest(tree_listing):
    return hashlib.sha256(tree_listing.encode("utf-8")).hexdigest()


if __name__ == "__main__":
    root=Path(__file__).resolve().parents[2]
    listing=subprocess.check_output(["git","ls-tree","-r","HEAD","--",*INPUTS],cwd=root,text=True)
    print(digest(listing))
