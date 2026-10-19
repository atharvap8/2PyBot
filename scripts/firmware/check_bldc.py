"""Compile/run production BLDC control, protocol and parameter regression checks."""
import argparse
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx",default="g++")
    parser.add_argument("--zig",action="store_true",help="Use zig c++ as the native compiler")
    args=parser.parse_args()
    output=ROOT/"build/bldc-checks";output.mkdir(parents=True,exist_ok=True)
    firmware=ROOT/"firmware/BaseLink";stubs=ROOT/"tests/stubs"
    compiler=[args.cxx]+(["c++"] if args.zig else [])
    tests={"test_bldc_control":[],"test_m0601":[firmware/"m0601.cpp",stubs/"runtime.cpp"],
           "test_bldc_params":[firmware/"params.cpp",stubs/"runtime.cpp"],
           "test_bldc_drive":[firmware/"hub_drive.cpp",firmware/"m0601.cpp",firmware/"params.cpp",stubs/"runtime.cpp"]}
    for name,extra in tests.items():
        binary=output/(name+(".exe" if sys.platform=="win32" else ""))
        subprocess.run(compiler+["-std=c++11","-Wall","-Wextra","-Werror","-I",str(stubs),"-I",str(firmware),
            str(ROOT/"tests"/(name+".cpp")),*map(str,extra),"-o",str(binary)],check=True)
        subprocess.run([str(binary)],check=True)

if __name__=="__main__":main()
