"""Validate continuation of a disabled MCU's existing bounded neutral search.

STOP retains the offset; BMODE/BAXIS/BDIR/BREF would reset it. Match the saved
run and fresh device fingerprint before skipping those configuration commands.
"""
import math


CONTROL_KEYS = ('kp','kd','kw','yaw_damping','body_axes','directions','voltage',
                'wheel_bound','body_rate_bound','capture_angle','attitude_tau',
                'neutral_gain','neutral_rate')
STATE_KEYS = ('App','BMode','Cal','IMU','Aligned','Ref','AxisMask','DirMask','NTune','NOn',
              'Axis1','Axis2','Axis3','D1','D2','D3','Map1','Map2','Map3',
              'PP1','PP2','PP3','Dir1','Dir2','Dir3')
FLOAT_FIELDS = {'NPit':.011,'NRol':.011,'NOff':.003,'NLimit':.001,
                'NRate':.001,'NGain':.0000006,'YawD':.011,'AttTau':.005,'BTrim':.011}


def validate_previous(previous, args, *, seed=False):
    settings, state = previous.get('settings',{}), previous.get('last_state',{})
    if not settings.get('neutral_learning') or previous.get('active_samples',0) < 1:
        raise RuntimeError('Resume requires an actual previous neutral-search run')
    for key in CONTROL_KEYS:
        if key not in settings or settings[key] != getattr(args,key):
            raise RuntimeError('Resume controller setting mismatch: '+key)
    base = previous.get('reference',{}).get('gravity',[])
    if len(base)!=3 or any(not math.isfinite(v) or abs(v-a)>1e-8
                          for v,a in zip(base,args.reference_gravity)):
        raise RuntimeError('Resume original reference mismatch')
    if any(key not in state for key in (*STATE_KEYS,*FLOAT_FIELDS)):
        raise RuntimeError('Resume saved device fingerprint incomplete')
    if any(state.get(k)!=v for k,v in {'BMode':'4','Cal':'1','Aligned':'111','Ref':'1',
                                      'NOn':'1','EN':'000','Bal':'0','Fault':'0'}.items()):
        raise RuntimeError('Resume requires confirmed disabled learned corner state')
    if not all(math.isfinite(float(state[k])) for k in FLOAT_FIELDS):
        raise RuntimeError('Resume non-finite saved target')
    limit = float(settings.get('neutral_limit', 2))
    if state.get('NTune') not in ('2','3') or (state['NTune']=='2' and limit!=2):
        raise RuntimeError('Resume unsupported learner version')
    if not 2 <= limit <= 5 or abs(float(state['NLimit'])-limit)>.001 or not 0 <= float(state['NOff']) <= limit+.001:
        raise RuntimeError('Resume original search boundary invalid')
    p,r=math.radians(float(state['NPit'])),math.radians(float(state['NRol']))
    target=[-math.sin(r)*math.cos(p),math.sin(p),math.cos(r)*math.cos(p)]
    angle=math.degrees(math.acos(max(-1,min(1,sum(a*b for a,b in zip(base,target))))))
    if abs(angle-float(state['NOff']))>.03:
        raise RuntimeError('Resume offset and target inconsistent')
    if angle>limit+.03 or angle>args.neutral_limit+.03:
        raise RuntimeError('Resume target outside original search boundary')
    if not seed and abs(limit-args.neutral_limit)>.001:
        raise RuntimeError('Resume search limit changed; use seed after reconfiguration')
    return float(state['NPit']),float(state['NRol'])


def matches_saved_state(current, previous):
    old=previous['last_state']
    if current.get('EN')!='000' or current.get('Bal')!='0' or current.get('Fault')!='0':
        return False
    if any(current.get(key)!=old.get(key) for key in STATE_KEYS):
        return False
    try:
        return all(math.isfinite(float(current[key])) and
                   abs(float(current[key])-float(old[key]))<=tolerance
                   for key,tolerance in FLOAT_FIELDS.items())
    except (KeyError,ValueError,TypeError):
        return False
