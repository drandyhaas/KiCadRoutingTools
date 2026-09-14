# Independent line-ending portability verification

Verified test-only revision: `7909abaccbe58221187cf41782a48e79af91debd`.
The one-file diff changes only the source identity assertion to normalize
CRLF to LF before hashing. Original fixture and production trees are unchanged.

```powershell
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 reviewer_portability_7909/verify_line_endings.py --root . --revision 7909abaccbe58221187cf41782a48e79af91debd --out reviewer_portability_7909/results
```

The runner loads the exact committed test Git object against independently
identical production trees in the isolated reviewer checkout. It executes all
four tests with the original source, then overrides the module's `SOURCE` with
an LF-only temporary copy and executes all four again. All eight executions
pass; no test is skipped. The executed test source, complete logs, exact command,
source copy and tree identities are retained in `results/`.

- Original CRLF working file SHA256:
  `165302e6a4f7aacdd64b3174df27ed8ddb19fd5f1f7effadd8d92205e25a120e`.
- LF-only copy and original Git blob SHA256:
  `a9945bb0940f79672b7c6e32b7a6b9d0b135bf78030e19fcdb88e65c2139903f`.

The LF copy equals the original Git blob byte-for-byte. Its only difference
from the measured Windows source is CRLF versus LF. The original source remains
byte-identical after both suites; no declaration, geometry or fixture was edited.
