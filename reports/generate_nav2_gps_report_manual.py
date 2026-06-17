import io

report_path = 'reports/nav2_gps_report.pdf'

lines = [
    'NJORD - Nav2 & GPS Report',
    '',
    'Summary: Concise summary of work on Nav2 and GPS integration for reliable, collision-free navigation.',
    '',
    'Key Objectives:',
    '- Robust Nav2 planning and safe local control',
    '- GPS integration for global waypoint following and coarse pose updates',
    '',
    'Work Completed:',
    '- Tuned global/local planners, costmaps, and recovery behaviors',
    '- Implemented GPS fusion with odometry (EKF) for improved global localization',
    '- Validated in sim and on hardware; created reproducible launch files',
    '',
    'Results: >95% collision-free runs in test course; primary failures due to GPS outages',
    '',
    'Next Steps: Improve GPS robustness (multi-constellation/RTK), final Nav2 tuning and runbook',
    '',
    'Contact: NODE Engineering Club - Nav2 Team'
]

# PDF primitive writer
buffer = io.BytesIO()

objs = []

def obj(n, data):
    return f"{n} 0 obj\n{data}\nendobj\n"

# We'll use Helvetica, place text on page
content_stream = 'BT\n/F1 14 Tf\n72 750 Td\n'
for i, line in enumerate(lines):
    safe = line.replace('(', '\\(').replace(')', '\\)')
    content_stream += f'({safe}) Tj\n0 -16 Td\n'
content_stream += 'ET\n'
content_bytes = content_stream.encode('latin1')

# Build objects
objs.append(obj(1, '<< /Type /Catalog /Pages 2 0 R >>'))
objs.append(obj(2, '<< /Type /Pages /Kids [3 0 R] /Count 1 >>'))
objs.append(obj(3, '<< /Type /Page /Parent 2 0 R /MediaBox [0 0 595 842] /Resources << /Font << /F1 4 0 R >> >> /Contents 5 0 R >>'))
objs.append(obj(4, '<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>'))
# content stream with proper Length
content_len = len(content_bytes)
objs.append(("5 0 obj\n<< /Length %d >>\nstream\n" % content_len).encode('latin1') + content_bytes + b"\nendstream\nendobj\n")

# Write header and objects while tracking offsets
buffer.write(b"%PDF-1.4\n%\xE2\xE3\xCF\xD3\n")
offsets = []
for o in objs:
    offsets.append(buffer.tell())
    if isinstance(o, bytes):
        buffer.write(o)
    else:
        buffer.write(o.encode('latin1'))

xref_pos = buffer.tell()
# xref
buffer.write(b"xref\n")
buffer.write(f"0 {len(objs)+1}\n".encode('latin1'))
buffer.write(b"0000000000 65535 f \n")
for off in offsets:
    buffer.write(f"{off:010d} 00000 n \n".encode('latin1'))

# trailer
buffer.write(b"trailer\n")
buffer.write(f"<< /Size {len(objs)+1} /Root 1 0 R >>\nstartxref\n{xref_pos}\n%%EOF\n".encode('latin1'))

with open(report_path, 'wb') as f:
    f.write(buffer.getvalue())

print(f'Wrote {report_path}')
