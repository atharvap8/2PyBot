# Revision numbering

After reaching minor revision 10, increment the minor number by **10** and keep
the patch at zero:

**3.10.0 → 3.20.0 → 3.30.0 → 3.40.0 → …**

The active development revision is **4.0.0**, a new major revision for the
M0601 BLDC hardware change. The v3 sequence stops at 3.40.0 on its existing
branches; v4 starts at 4.0.0. Numbers such as 3.11.0, 3.12.0,
3.13.0 or 3.21.0 are not valid new revisions. Build checks enforce this for the
firmware wrapper, newly published images, Android and release workflows.

```sh
python scripts/versioning.py --check
python scripts/versioning.py --next
```

Existing prototype artifacts/recordings keep their original identifiers and
hashes for traceability and verified rollback. They are superseded by the
3.20.0 revision, rather than relabelled as newly compiled binaries.
