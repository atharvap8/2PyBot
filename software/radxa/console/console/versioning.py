"""Project revision policy: ... 3.9.0, 3.10.0, 3.20.0, 3.30.0 ..."""
import re


def validate_version(value,allow_legacy=False,allow_prerelease=False):
    match=re.fullmatch(r"(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)(-rc\.[1-9]\d*)?",value)
    if not match or match[4] and not allow_prerelease:
        raise ValueError("Use major.minor.patch, optionally -rc.N for release candidates")
    major,minor,patch=map(int,match.group(1,2,3))
    if not allow_legacy and minor>=10 and (minor%10!=0 or patch!=0):
        raise ValueError("After 3.10.0 use increments of ten: 3.20.0, 3.30.0, 3.40.0; no 3.11/3.12 numbering")
    return major,minor,patch


def next_revision(value):
    major,minor,_=validate_version(value)
    return f"{major}.{minor+10 if minor>=10 else minor+1}.0"
