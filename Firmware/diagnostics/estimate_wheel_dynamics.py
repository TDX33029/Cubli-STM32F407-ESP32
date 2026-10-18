"""Offline wheel/CAD estimates. Every unmeasured assumption remains explicit."""
import hashlib
import json
import math
import os
from pathlib import Path

import ezdxf
import numpy as np
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

ROOT = Path(__file__).resolve().parents[2]
FOLDER = Path(__file__).resolve().parent / '2026-10-14'
CAD = ROOT / 'Mechanical/2D/动量轮CAD.dxf'
RHO = 7930.0
THICKNESS = .005


def annulus(ro, ri):
    mass = math.pi * (ro**2 - ri**2) * THICKNESS * RHO
    return {'mass_kg': mass, 'polar_inertia_kg_m2': mass * (ro**2 + ri**2) / 2}


def flatten_polyline(entity, tolerance_mm):
    vertices = list(entity.get_points())
    points = []
    for index, vertex in enumerate(vertices):
        x0, y0, _, _, bulge = vertex
        x1, y1 = vertices[(index + 1) % len(vertices)][:2]
        if abs(bulge) < 1e-12:
            points.append((x0, y0))
            continue
        sweep = 4 * math.atan(bulge)
        dx, dy = x1 - x0, y1 - y0
        cx = (x0 + x1) / 2 - dy / (2 * math.tan(sweep / 2))
        cy = (y0 + y1) / 2 + dx / (2 * math.tan(sweep / 2))
        radius = math.hypot(x0 - cx, y0 - cy)
        start = math.atan2(y0 - cy, x0 - cx)
        step = 2 * math.acos(max(-1, min(1, 1 - tolerance_mm / radius)))
        count = max(2, math.ceil(abs(sweep) / step))
        points.extend((cx + radius * math.cos(start + sweep * k / count),
                       cy + radius * math.sin(start + sweep * k / count)) for k in range(count))
    return Polygon(points)


def circle(x, y, radius, tolerance_mm):
    step = 2 * math.acos(1 - tolerance_mm / radius)
    return Point(x, y).buffer(radius, resolution=max(16, math.ceil(math.pi / (2 * step))))


def signed_moments(coordinates):
    xy = np.asarray(coordinates)
    a, b = xy[:-1], xy[1:]
    cross = a[:, 0] * b[:, 1] - b[:, 0] * a[:, 1]
    area = float(cross.sum() / 2)
    polar = float(np.sum(cross * (a[:, 0]**2 + a[:, 0]*b[:, 0] + b[:, 0]**2 +
                                  a[:, 1]**2 + a[:, 1]*b[:, 1] + b[:, 1]**2)) / 12)
    return abs(area), abs(polar)


def material_properties(geometry):
    polygons = [geometry] if geometry.geom_type == 'Polygon' else list(geometry.geoms)
    area = polar = 0.0
    for polygon in polygons:
        ar, jp = signed_moments(polygon.exterior.coords)
        area += ar
        polar += jp
        for hole in polygon.interiors:
            ar, jp = signed_moments(hole.coords)
            area -= ar
            polar -= jp
    return {'area_mm2': area, 'mass_kg': area * 1e-6 * THICKNESS * RHO,
            'polar_inertia_kg_m2': polar * 1e-12 * THICKNESS * RHO,
            'valid': geometry.is_valid, 'connected_components': len(polygons)}


def cad_material(document, center_hole_diameter_mm, tolerance_mm):
    cutouts = []
    for entity in document.modelspace():
        if entity.dxftype() == 'LWPOLYLINE':
            if not entity.closed:
                raise ValueError('Open CAD contour')
            cutouts.append(flatten_polyline(entity, tolerance_mm))
        elif entity.dxftype() == 'CIRCLE':
            x, y, _ = entity.dxf.center
            radius = entity.dxf.radius
            if radius > 30:
                outer = circle(x, y, radius, tolerance_mm)
            else:
                if abs(x) < 1e-6 and abs(y) < 1e-6:
                    radius = center_hole_diameter_mm / 2
                cutouts.append(circle(x, y, radius, tolerance_mm))
        else:
            raise ValueError('Unsupported CAD entity: ' + entity.dxftype())
    return outer.difference(unary_union(cutouts))


def calculate():
    document = ezdxf.readfile(CAD)
    if document.units != 4:
        raise ValueError('CAD is not in millimeters')
    outer = annulus(.033, .0255)
    inner = annulus(.010, .008)
    known_mass = outer['mass_kg'] + inner['mass_kg']
    known_inertia = outer['polar_inertia_kg_m2'] + inner['polar_inertia_kg_m2']
    coarse = cad_material(document, 12, .001)
    drawing = cad_material(document, 12, .0001)
    user_hole = cad_material(document, 16, .0001)
    cad12, cad16 = material_properties(drawing), material_properties(user_hole)
    coarse_props = material_properties(coarse)
    identification = json.loads((FOLDER / 'motor_pulse_identification.json').read_text())
    acceleration_gain = identification['voltage_acceleration_gain_rad_s2_per_V']
    torque_gain = acceleration_gain * known_inertia
    body_side = .070
    geometric_com_height = body_side / math.sqrt(2)
    scenarios = []
    for motor_mass_g in (0, 20, 30, 40):
        total_mass = 3 * known_mass + .050 + 3 * motor_mass_g / 1000
        gravity_slope = total_mass * 9.81 * geometric_com_height
        scenarios.append({
            'assumed_each_motor_mass_g': motor_mass_g,
            'total_mass_kg_excluding_spokes_fasteners': total_mass,
            'uniform_cube_edge_inertia_kg_m2': 2 * total_mass * body_side**2 / 3,
            'geometric_gravity_torque_slope_Nm_per_rad': gravity_slope,
            'gravity_torque_Nm': {str(angle): gravity_slope * math.sin(math.radians(angle)) for angle in (1, 3, 5, 10, 20)},
            'gravity_only_phase_voltage_V': {str(angle): gravity_slope * math.sin(math.radians(angle)) / torque_gain for angle in (1, 3, 5, 10, 20)},
        })
    cad_scenarios = []
    for name, wheel in [('drawing_center12',cad12),('hypothetical_center16',cad16)]:
        for motor_mass_g in (20,30,40):
            mass = 3*wheel['mass_kg']+.050+3*motor_mass_g/1000
            gain = acceleration_gain*wheel['polar_inertia_kg_m2']
            slope = mass*9.81*geometric_com_height
            # Illustrative free-body model. Remove the controlled wheel's axial
            # inertia from the locked-wheel cube inertia before forming body dynamics.
            total_inertia = 2*mass*body_side**2/3
            body_inertia = total_inertia-wheel['polar_inertia_kg_m2']
            a=slope/body_inertia;b=gain/body_inertia;c=acceleration_gain;ke=.024
            desired=np.real(np.poly([-10+10j,-10-10j,-1]))
            alpha,beta,gamma=desired[1:]
            h=-gamma/(a*c)
            kw=h-ke;kp=(beta+a)/b;kd=(alpha-(b+c)*h)/b
            plant=np.array([[0,1,0],[a,0,-b*ke],[a,0,-(b+c)*ke]])
            control=np.array([0,b,b+c])
            closed=plant-np.outer(control,[kp,kd,kw])
            poles=np.linalg.eigvals(closed)
            if not np.allclose(np.poly(closed),desired,rtol=1e-8,atol=1e-8):
                raise ArithmeticError('State-space and polynomial gain calculations disagree')
            cad_scenarios.append({'wheel_case':name,'assumed_each_motor_mass_g':motor_mass_g,
                                  'total_mass_kg_excluding_fasteners':mass,
                                  'torque_gain_Nm_per_V':gain,
                                  'geometric_gravity_torque_slope_Nm_per_rad':slope,
                                  'gravity_only_phase_voltage_V':{str(angle):slope*math.sin(math.radians(angle))/gain for angle in (1,3,5,10,20)},
                                  'offline_feedback_candidate':{
                                      'uniform_locked_cube_edge_inertia_kg_m2':total_inertia,
                                      'body_edge_inertia_excluding_controlled_wheel_spin_kg_m2':body_inertia,
                                      'plant_a_b_c':[a,b,c],
                                      'firmware_body_direction_assumed':-1,
                                      'K_theta':float(kp),'K_body_rate':float(kd),'K_wheel_signed':float(kw),'K_integral':0,
                                      'predicted_full_gain_poles':[{'real':float(z.real),'imag':float(z.imag)} for z in poles],
                                      'applies_after_handover':True,
                                      'note':'OFFLINE ONLY: assumes geometric neutral COM and positive body response to positive q voltage. No voltage saturation, friction, sensor delay or current limit. Must not be installed as validated gains.'}})
    assembly_mass=cad16['mass_kg']+.030
    illustrative_total=3*assembly_mass+.050
    com_examples=[]
    for board_mass in (0,.020,.050):
        x=-assembly_mass*body_side/2/illustrative_total
        z=(-assembly_mass+board_mass)*body_side/2/illustrative_total
        com_examples.append({'assumed_board_mass_on_face1_kg':board_mass,
                             'remaining_frame_mass_at_center_kg':.050-board_mass,
                             'COM_xz_m':[x,z],
                             'modeled_neutral_angle_deg':math.degrees(math.atan2(body_side/2-x,body_side/2+z))})
    result = {
        'source': 'User dimensions and material, 2026-10-14; no hardware was powered for this calculation.',
        'user_inputs': {'cube_side_mm': 70, 'outer_ring_OD_ID_mm': [66, 51],
                        'inner_ring_OD_ID_mm': [20, 16], 'thickness_mm': 5,
                        'material': '304 stainless steel', 'density_assumed_kg_m3': RHO,
                        'motor': '2804 220KV', 'frame_and_mainboard_mass_kg_approx': .050,
                        'motor_mass_measured': False, 'total_mass_measured': False},
        'annuli_only': {'outer': outer, 'inner': inner, 'one_mass_kg': known_mass,
                       'one_inertia_kg_m2': known_inertia, 'three_mass_kg': 3 * known_mass,
                       'nominal_mass_subtotal_kg_before_motors_spokes_and_cutouts': 3 * known_mass + .050},
        'cad_crosscheck': {'file': str(CAD.relative_to(ROOT)), 'sha256': hashlib.sha256(CAD.read_bytes()).hexdigest(),
                          'matches': ['outer OD66', 'outer ring ID51', 'hub OD20', 'thickness supplied by user'],
                          'drawing_center_hole_mm': 12, 'bolt_pattern_PCD_mm': 16,
                          'bolt_holes': '4 x diameter 2.5mm', 'spokes_width_mm': 2.5,
                          'mismatch': 'User described a 16mm center opening; drawing center opening is 12mm. 16mm is also the drawing bolt PCD. Actual interpretation must be checked.',
                          'original_drawing_center12': cad12,
                          'hypothetical_center16_same_spokes_and_bolts': cad16,
                          'mass_relative_change_at_10x_finer_tolerance': abs(cad12['mass_kg'] / coarse_props['mass_kg'] - 1),
                          'note': 'CAD estimates include spokes and screw holes; center16 is a hypothetical edit, not a claim about the installed wheel. Original CAD was not modified.'},
        'pulse_based_torque_estimate': {'acceleration_gain_rad_s2_per_V': acceleration_gain,
                                      'annuli_only_torque_gain_Nm_per_V': torque_gain,
                                      'phase_bemf_used_V_per_rad_s': .024,
                                      'note': 'Approximation from short filtered-speed pulses; not measured current or torque. Inertia and mass exclude unconfirmed spokes/rotor.'},
        'body_mass_scenarios': scenarios,
        'cad_body_mass_scenarios':cad_scenarios,
        'asymmetric_COM_examples':{
            'assumptions':'Hypothetical center16 CAD wheel, each motor 30g, equal wheel/motor assemblies at face centers 2(-X),3(Y),6(-Z), 50g other mass split between face1(+Z) and geometric center; selected Y edge at +X/-Z.',
            'examples':com_examples,
            'note':'These are placement examples, not bounds or measured neutral angles. Real motor depth, board/frame distribution and cable loads are unknown; no reference should be programmed from this table alone.'},
        'limitations': ['Frame mass is approximate; motors, spokes, screws and exact mounting positions are not measured.',
                        'Uniform cube inertia and geometric COM height are illustrative; asymmetric wheel placement changes the true COM.',
                        'Gravity-only voltage ignores wheel speed, friction, induction, driver current limits and transient dynamics.',
                        'Same-direction acceleration margin decreases with phase back EMF. Opposite-direction braking requires a separate calculation.',
                        '20 degrees is the configured shutdown threshold; it is not a demonstrated recovery angle.'],
    }
    return result, drawing


def export_plot(data, drawing):
    os.environ['QT_QPA_PLATFORM'] = 'windows'
    from PyQt5.QtCore import Qt
    from PyQt5.QtGui import QFont
    from PyQt5.QtWidgets import QApplication
    import pyqtgraph as pg
    import pyqtgraph.exporters
    app = QApplication([])
    app.setFont(QFont('Microsoft YaHei', 9))
    pg.setConfigOptions(background='w', foreground='#334155', antialias=True)
    w = pg.GraphicsLayoutWidget()
    w.setAttribute(Qt.WA_DontShowOnScreen, True)
    w.resize(1400, 650)
    w.addLabel('70mm Cubli：动量轮图纸核对与条件力矩估算（设备断电）', colspan=2, size='15pt', bold=True)
    left = w.addPlot(row=1, col=0, title='原DXF：中心孔Ø12，四孔分布圆PCD16')
    left.setAspectLocked(True)
    left.setLabel('bottom', 'X', units='mm')
    left.setLabel('left', 'Y', units='mm')
    from PyQt5.QtGui import QPainterPath, QBrush, QColor
    from PyQt5.QtWidgets import QGraphicsPathItem
    path=QPainterPath();path.setFillRule(Qt.OddEvenFill)
    for contour in [drawing.exterior,*drawing.interiors]:
        xy = np.asarray(contour.coords)
        path.moveTo(float(xy[0, 0]), float(xy[0, 1]))
        for x, y in xy[1:]: path.lineTo(float(x), float(y))
        path.closeSubpath()
    item=QGraphicsPathItem(path);item.setBrush(QBrush(QColor('#cbd5e1')));item.setPen(pg.mkPen('#475569',width=.15));left.addItem(item)
    phi = np.linspace(0, 2 * math.pi, 500)
    left.plot(8*np.cos(phi), 8*np.sin(phi), pen=pg.mkPen('#f97316', width=1.4, style=Qt.DashLine))
    left.setXRange(-36, 36);left.setYRange(-36, 36)
    right = w.addPlot(row=1, col=1, title='条件示例：图纸辐条+Ø16中心孔，电机各假设30g，总重约335g')
    right.addLegend(offset=(5,5));right.showGrid(x=True,y=True,alpha=.2)
    right.setLabel('bottom','距真实平衡点的倾角',units='°');right.setLabel('left','力矩',units='N·m')
    angle = np.linspace(0,25,300)
    case = next(c for c in data['cad_body_mass_scenarios'] if c['assumed_each_motor_mass_g']==30 and c['wheel_case']=='hypothetical_center16')
    right.plot(angle,case['geometric_gravity_torque_slope_Nm_per_rad']*np.sin(np.radians(angle)),pen=pg.mkPen('#334155',width=2),name='重力力矩（几何重心模型）')
    gain=case['torque_gain_Nm_per_V']
    for volts,color in [(2.5,'#2563eb'),(4.5,'#059669')]:
        for speed,style in [(0,Qt.SolidLine),(80,Qt.DashLine)]:
            available=gain*max(0,volts-.024*speed)
            right.plot(angle,np.full_like(angle,available),pen=pg.mkPen(color,width=1.7,style=style),name=f'{volts}V，同向加速，{speed}rad/s')
    right.setXRange(0,25);right.setYRange(0,.075)
    w.addLabel('橙色虚线：用户描述的Ø16中心孔，需核实是否指PCD16。右图采用DXF辐条/孔布局的条件模型，电机未称重，未计转子与电流限制。',row=2,col=0,colspan=2)
    w.addLabel('20°停机设置已保留；真实恢复范围需在自由响应下实测，不能由本估算保证。',row=3,col=0,colspan=2)
    w.show();app.processEvents();exporter=pg.exporters.ImageExporter(w.scene());exporter.parameters()['width']=1600
    exporter.export(str(FOLDER/'wheel_cad_and_torque_estimate.png'));w.close()


if __name__ == '__main__':
    data, material = calculate()
    (FOLDER/'wheel_and_body_estimate.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    export_plot(data,material)
    print(json.dumps({'annuli_only':data['annuli_only'],'cad_crosscheck':data['cad_crosscheck'],
                      'motor30g_case':data['body_mass_scenarios'][2]},ensure_ascii=False,indent=2))
