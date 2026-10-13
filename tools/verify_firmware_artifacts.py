"""Verify built images, identities, linker budgets, and retain auditable ARM evidence.

This checks final linked artifacts, not MCU wall-clock timing or stack watermarks.
"""
from pathlib import Path
import argparse, hashlib, json, re, struct, subprocess

args=argparse.ArgumentParser()
args.add_argument('--artifacts', default=str(Path(__file__).resolve().parent.parent/'firmware_artifacts'))
args.add_argument('--output', default=None)
opt=args.parse_args()
art=Path(opt.artifacts).resolve()
repo=art.parent
out=Path(opt.output).resolve() if opt.output else art
out.mkdir(parents=True,exist_ok=True)
targets=['Core-STM32F4-fw','motor-35','motor-42','motor-57','motor-gripper']
result={'artifact_directory':str(art),'targets':{},'hardware_measurements':False,
        'scope':'Linked memory/identity/HEX checks and static ARM call paths; not a measured whole-ISR WCET or stack watermark.'}
identity=None
for name in targets:
    project='dummy-ref-core-fw' if name==targets[0] else 'dummy-motor-fw'
    mf=(art/(name+'.manifest.txt')).read_text(encoding='utf-8')
    fields=dict(line.split('=',1) for line in mf.splitlines() if '=' in line and '\t' not in line)
    current=tuple(fields[k] for k in ('release','buildset','utc_epoch','source_sha256'))
    if identity is None: identity=current
    assert current==identity, (name,'mixed build identity')
    inputs=0
    for line in mf.splitlines():
        if '\t' not in line:continue
        relative,digest=line.split('\t',1)
        if len(digest)!=64: continue
        assert hashlib.sha256((repo/'firmware_new'/relative).read_bytes()).hexdigest()==digest, relative
        inputs+=1
    assert inputs>490
    binary=(art/(name+'.bin')).read_bytes()
    kind={'Core-STM32F4-fw':0,'motor-35':35,'motor-42':42,'motor-57':57,'motor-gripper':8}[name]
    words=[0x31565746,0x701|(kind<<16),int(identity[0]),int(identity[2]),int(identity[1][:8],16),int(identity[1][8:],16),
           *[int(identity[3][i:i+8],16) for i in range(0,64,8)]]
    assert binary.count(struct.pack('<14I',*words))==1,(name,'embedded identity')
    flash_limit=(896 if kind==0 else 95)*1024
    assert len(binary)<=flash_limit
    upper=0;count_bytes=0;seen=set()
    for line in (art/(name+'.hex')).read_text().splitlines():
        assert line.startswith(':')
        raw=bytes.fromhex(line[1:]);assert len(raw)==raw[0]+5 and sum(raw)%256==0
        if raw[3]==4:upper=int.from_bytes(raw[4:6],'big')<<16
        elif raw[3]==0:
            address=upper+int.from_bytes(raw[1:3],'big'); n=raw[0]
            assert 0x08000000<=address and address+n<=0x08000000+flash_limit
            offset=address-0x08000000
            assert binary[offset:offset+n]==raw[4:4+n],(name,'HEX/BIN mismatch')
            assert not any(a in seen for a in range(address,address+n))
            seen.update(range(address,address+n));count_bytes+=n
    elf=art/(name+'.elf')
    def run(*cmd):return subprocess.check_output(list(cmd)+[str(elf)],text=True)
    size=run('arm-none-eabi-size','-A')
    nm=run('arm-none-eabi-nm','-S','-n','-C')
    dis=run('arm-none-eabi-objdump','-d','-C')
    (out/(name+'.size.txt')).write_text(size)
    (out/(name+'.symbols.txt')).write_text(nm)
    (out/(name+'.disassembly.txt')).write_text(dis)
    symbols={}
    for line in nm.splitlines():
        m=re.match(r'([0-9a-f]+)(?: ([0-9a-f]+))? ([A-Za-z]) (.+)',line)
        if m:symbols[m[4]]={'address':int(m[1],16),'bytes':int(m[2],16) if m[2] else None,'kind':m[3]}
    sections={m[1]:{'bytes':int(m[2]),'address':int(m[3])} for m in re.finditer(r'^(\S+)\s+(\d+)\s+(\d+)\s*$',size,re.M)}
    ram_static=sum(v['bytes'] for k,v in sections.items() if k in ('.data','.bss'))
    if kind==0:
        for stack,expected in (('commandTaskStack',4096),('usbServerStack',6144),('uartServerStack',6144)):
            assert symbols[stack]['bytes']==expected,(name,stack)
        heap=symbols['__HeapLimit']['address']-symbols['__HeapBase']['address']
        assert heap>=0x3c00,('core reserved C heap',heap)
        ram_limit=128*1024
        stack_reserved=0x800
    else:
        ram_limit=20*1024
        stack_reserved=symbols['_Min_Stack_Size']['address'];assert stack_reserved==4096
        heap_floor=symbols.get('_end',symbols.get('end',symbols['_ebss']))['address']
        heap=symbols['_estack']['address']-stack_reserved-heap_floor
        assert heap>=512,('motor C heap',name,heap)
    stack_usage=[]
    build=art/'.build'/project
    for file in build.rglob('*.su'):
        if kind and ('CMakeFiles/'+name+'.elf.dir') not in file.as_posix():continue
        for line in file.read_text(errors='replace').splitlines():
            if any(x in line for x in ('Stream','stream','Can','CAN_','Tim4Callback','CloseLoop','UpdateAngle','Main()',
                                       'ThreadControlLoopFixUpdate','ASCII_protocol','OnUsbAsciiCmd','ADC','Transmit')):
                stack_usage.append(line)
    (out/(name+'.stack-frames.txt')).write_text('\n'.join(stack_usage),encoding='utf-8')
    functions={}
    for block in re.split(r'(?=^[0-9a-f]+ <)',dis,flags=re.M):
        m=re.match(r'[0-9a-f]+ <(.+)>:',block)
        if m:functions[m[1]]=block
    hot={}
    for fn,block in functions.items():
        if any(fn.startswith(p) for p in ('MotorStream::Curve::Tick(', 'MotorStream::Runtime::Tick(',
                                          'Motor::Controller::TickStream(', 'Tim4Callback20kHz',
                                          'Motor::Controller::CloseLoopControlTick(', 'MT6816Base::UpdateAngle(')):
            calls=re.findall(r'\t(?:bl|blx)\s+[^<]*<([^>]+)>',block)
            forbidden=re.findall(r'<(__aeabi_[df][^>]*|malloc|operator new[^>]*)>',block)
            if fn.startswith(('MotorStream::Curve::Tick(','MotorStream::Runtime::Tick(','Motor::Controller::TickStream(')):
                assert not forbidden,(name,fn,forbidden)
            hot[fn]={'static_instruction_lines':len(re.findall(r'^\s+[0-9a-f]+:\s+[0-9a-f ]+\s+\S+',block,re.M)),
                     'direct_calls':calls,'float_or_allocation_calls':forbidden}
    if kind: assert len(hot)>=3
    result['targets'][name]={'flash_bytes':len(binary),'flash_limit':flash_limit,'flash_spare_bytes':flash_limit-len(binary),
        'ram_static_bytes':ram_static,'ram_limit':ram_limit,'msp_reserved_bytes':stack_reserved,
        'c_heap_address_capacity':heap,'hex_loaded_bytes':count_bytes,'source_files_verified':inputs,
        'sections':sections,'hot_paths':hot,'stack_frames_count':len(stack_usage),
        'sha256':{ext:hashlib.sha256((art/(name+'.'+ext)).read_bytes()).hexdigest() for ext in ('bin','hex','elf','map')}}
result.update(zip(('release','buildset','epoch','source_sha256'),identity))
(out/'verification.json').write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k!='targets'},indent=2))
print(json.dumps({k:{f:v[f] for f in ('flash_bytes','flash_spare_bytes','ram_static_bytes','c_heap_address_capacity','msp_reserved_bytes')} for k,v in result['targets'].items()},indent=2))
