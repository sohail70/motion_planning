import re
import sys

def parse_hotspots(lines):
    """
    Parse lines like:
      113,236,911 ( 1.81%)  ./malloc/...:_int_malloc
    into a dict {function_name: count}
    """
    pattern = re.compile(r'^\s*([\d,]+)\s+\([\s\d\.%]+\)\s+.*[:/]([^:\s]+)\s*$')
    stats = {}
    for line in lines:
        m = pattern.match(line)
        if not m: 
            continue
        count = int(m.group(1).replace(',', ''))
        fn    = m.group(2)
        stats[fn] = stats.get(fn, 0) + count
    return stats

def compare(a, b):
    """
    Given two dicts a=new, b=old, compute delta and ratio.
    """
    rows = []
    all_fns = set(a) | set(b)
    for fn in all_fns:
        new = a.get(fn, 0)
        old = b.get(fn, 0)
        delta = new - old
        ratio = (new / old) if old else float('inf')
        rows.append((delta, ratio, new, old, fn))
    # Sort by biggest increase
    rows.sort(reverse=True, key=lambda r: r[0])
    print(f"{'Δinstr':>12} {'ratio':>8} {'new':>12} {'old':>12} function")
    for delta, ratio, new, old, fn in rows[:20]:
        print(f"{delta:12,d} {ratio:8.2f} {new:12,d} {old:12,d} {fn}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python compare.py new.txt old.txt")
        sys.exit(1)

    with open(sys.argv[1]) as f:
        new_stats = parse_hotspots(f)
    with open(sys.argv[2]) as f:
        old_stats = parse_hotspots(f)

    compare(new_stats, old_stats)