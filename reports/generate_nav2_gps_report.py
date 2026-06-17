from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_CENTER
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer

report_path = "reports/nav2_gps_report.pdf"

styles = getSampleStyleSheet()
styles.add(ParagraphStyle(name='TitleCenter', parent=styles['Heading1'], alignment=TA_CENTER))
styles.add(ParagraphStyle(name='SubHeading', parent=styles['Heading2'], spaceBefore=6))

title = "NJORD — Nav2 & GPS Report"

content = [
    Paragraph(title, styles['TitleCenter']),
    Spacer(1, 12),

    Paragraph("Summary", styles['SubHeading']),
    Paragraph("Concise summary of work on the Nav2 navigation stack and GPS integration for the NJORD competition. Focused on reliable, collision-free navigation and robust global positioning.", styles['Normal']),
    Spacer(1, 6),

    Paragraph("Key Objectives", styles['SubHeading']),
    Paragraph("- Deliver a robust Nav2-based planner setup for safe path following and obstacle avoidance.\n- Integrate GPS for improved global localization and waypoint navigation.", styles['Normal']),
    Spacer(1, 6),

    Paragraph("Work Completed", styles['SubHeading']),
    Paragraph("- Tuned Nav2 planners (global and local), costmaps, and recovery behaviors for high safety margins.\n- Implemented dynamic footprint and reactive replanning to avoid collisions.\n- Integrated GPS into the localization pipeline for coarse global pose updates and waypoint following.\n- Validated in simulation and hardware; organized launch files for reproducible deployment.", styles['Normal']),
    Spacer(1, 6),

    Paragraph("Technical Notes", styles['SubHeading']),
    Paragraph("- Nav2: adjusted planner weights, inflation/costmap parameters, and controller bounds to favor safety over minimal path length.\n- GPS: fused with odometry and AMCL via an extended Kalman filter for robustness against GPS noise.\n- Perception: vision and range sensors feed the costmap to prevent planning into detected obstacles.", styles['Normal']),
    Spacer(1, 6),

    Paragraph("Results & Metrics", styles['SubHeading']),
    Paragraph("- >95% collision-free runs on the test course; issues mainly from extreme GPS outages or sensor occlusion.\n- Perception-to-planning loop maintains operational latency suitable for reactive avoidance.", styles['Normal']),
    Spacer(1, 6),

    Paragraph("Next Steps", styles['SubHeading']),
    Paragraph("- Improve GPS robustness (multi-constellation, RTK where available) and add fallbacks for outages.\n- Further tune Nav2 parameters for specific competition arenas and record a runbook for event day.", styles['Normal']),
    Spacer(1, 12),

    Paragraph("Contact: NODE Engineering Club — Nav2 Team. For demos or logs, request artifacts.", styles['Normal'])
]

pdf = SimpleDocTemplate(report_path, pagesize=A4, rightMargin=36, leftMargin=36, topMargin=36, bottomMargin=36)
pdf.build(content)
print(f"Wrote {report_path}")
