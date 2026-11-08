#!/usr/bin/env python3
# compose.py: builds a shared file from upstream/master plus the added/removed
# lines of master that belong to the selected units (classification by keywords).
# Lines matching the exclusion list are never taken.  Unclassified changed lines abort.
import difflib, subprocess, sys, re

BASE = 'b7f483f3a1d'   # upstream/master the printer branches are based on

UNITS = [
    (1, r'etibuffer|CENTRONICS_PB642|"pb642"|centronics/pb642|layout/pb642|"emu/layout", "pb642"|pb642\.h"'),
    (2, r'pdfprinter'),
    (3, r'hplj3p|hplj4l|hp_lj3p|hp_lj4l|HP_LJ3P|HP_LJ4L|HP LaserJet'),
    (4, r'zxprinter|ZX81|zx81|alphacom32|ZXPRINTER'),
    (5, r'okimate20|okimat20|OKIMATE20|mcs820|MCS820|mcs810'),
    (6, r'star_nl10|STAR_NL10|nixdorf_md16|NIXDORF_MD16|"nl10"|"md16"'),
    (7, r'mx80|MX80|epson_fx80|EPSON_FX80|jx80|JX80|fx85|FX85|rx80|RX80'),
    (8, r'lx86|LX86|lx800|LX800|lx810l|LX810L|ap2000|AP2000|ex800|EX800|fx880|FX880|ap3250|AP3250'),
    (9, r'gem10x|GEMINI10X|200gx|200GX|bx80|BX80'),
    (10, r'iwriter2|IMAGEWRITER2'),
]
EXCLUDE = r'calcuscribe|cscribed|k215|ncr_gfx|ncr9010|NCR 9010|quaderno|skeleton/pb642|^pb642$|p6plus|P6PLUS|kxp2130|KXP2130|hplj3si|ljhost'

def classify(line):
    s = line.rstrip('\n')
    if re.search(EXCLUDE, s):
        return 99
    for u, pat in UNITS:
        if re.search(pat, s):
            return u
    if s.strip() in ('', '}', 'end', '{', 'files {', 'dependency {', 'custombuildtask {') or re.fullmatch(r'-+|--', s.strip()):
        return 0   # structural: decided by its block
    return None

def show(ref, path):
    r = subprocess.run(['git', 'show', f'{ref}:{path}'], capture_output=True, text=True)
    return r.stdout.splitlines(True) if r.returncode == 0 else []

def compose(unit, path):
    sel = unit if isinstance(unit, (set, frozenset)) else set(range(1, unit + 1))
    a = show(BASE, path)
    b = show('origin/master', path)
    out = []
    sm = difflib.SequenceMatcher(None, a, b, autojunk=False)
    for tag, i1, i2, j1, j2 in sm.get_opcodes():
        if tag == 'equal':
            out += a[i1:i2]
            continue
        dels, adds = a[i1:i2], b[j1:j2]
        units = [classify(l) for l in dels + adds]
        known = [u for u in units if u not in (None, 0, 99)]
        bad = [l for l, u in zip(dels + adds, units) if u is None]
        if bad:
            sys.exit(f'{path}: unclassified lines:\n' + ''.join(bad))
        block = min(known) if known else 99
        for l in dels:
            u = classify(l)
            u = block if u == 0 else u
            if u not in sel:
                out.append(l)          # removal not (yet) applied
        for l in adds:
            u = classify(l)
            u = block if u == 0 else u
            if u in sel:
                out.append(l)
    return ''.join(out)
