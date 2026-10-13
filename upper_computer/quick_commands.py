"""Editable raw UDP command definitions; no MCU protocol or execution implied."""
import ipaddress
import json
from pathlib import Path
import struct
import uuid

FORMATS = ('UTF-8','GBK','HEX','JSON','Binary')
ENDINGS = ('None','LF','CRLF')
FIELD_TYPES = {'uint8':'B','int8':'b','uint16':'H','int16':'h','uint32':'I','int32':'i',
               'uint64':'Q','int64':'q','float32':'f','float64':'d'}

def new_command(name='新命令'):
    return dict(id=uuid.uuid4().hex,name=name,format='UTF-8',content='',note='',ending='None',byte_order='little')

def defaults():
    return dict(version=1,inherit_target=True,target_ip='',target_port=6666,commands=[new_command()])

def validate_config(value):
    if not isinstance(value,dict) or type(value.get('version')) is not int or value['version'] != 1:
        raise ValueError('命令文件必须是 version=1 的 JSON 对象')
    if type(value.get('inherit_target')) is not bool:
        raise ValueError('inherit_target 必须为布尔值')
    if not isinstance(value.get('target_ip'),str) or type(value.get('target_port')) is not int or not 1 <= value['target_port'] <= 65535:
        raise ValueError('命令目标地址或端口格式不正确')
    if not isinstance(value.get('commands'),list):
        raise ValueError('commands 必须是列表')
    commands,ids = [],set()
    for item in value['commands']:
        if not isinstance(item,dict):
            raise ValueError('每个条目必须是对象')
        command = {key:item.get(key,default) for key,default in new_command().items()}
        if any(not isinstance(v,str) for v in command.values()):
            raise ValueError('条目字段必须是文字')
        if command['format'] not in FORMATS or command['ending'] not in ENDINGS or command['byte_order'] not in ('little','big'):
            raise ValueError('命令格式、行尾或字节序不受支持')
        if not command['id'] or command['id'] in ids:
            command['id'] = uuid.uuid4().hex
        ids.add(command['id'])
        commands.append(command)
    return dict(version=1,inherit_target=value['inherit_target'],target_ip=value['target_ip'],
                target_port=value['target_port'],commands=commands)

def load_config(path):
    return validate_config(json.loads(Path(path).read_text('utf-8-sig')))

def save_config(path, value):
    value = validate_config(value)
    path = Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    temporary = path.with_name(path.name+'.tmp_'+uuid.uuid4().hex)
    try:
        temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')
        temporary.replace(path)
    finally:
        if temporary.exists():
            temporary.unlink()

def encode_command(command):
    content,mode = command['content'],command['format']
    if mode == 'HEX':
        try:
            payload = bytes.fromhex(content)
        except ValueError as exc:
            raise ValueError('HEX 内容应为成对十六进制字节，例如 AA 55 01') from exc
    elif mode == 'Binary':
        fields = json.loads(content)
        if not isinstance(fields,list):
            raise ValueError('Binary 内容必须是 type/value 字段的 JSON 数组')
        parts = []
        for field in fields:
            if not isinstance(field,dict) or field.get('type') not in FIELD_TYPES or type(field.get('value')) not in (int,float):
                raise ValueError('Binary 字段必须包含有效 type 和数值 value')
            try:
                parts.append(struct.pack(('<' if command['byte_order']=='little' else '>')+
                                         FIELD_TYPES[field['type']],field['value']))
            except (struct.error,OverflowError) as exc:
                raise ValueError('Binary 数值超出类型范围或整数类型收到小数') from exc
        payload = b''.join(parts)
    elif mode in ('UTF-8','GBK','JSON'):
        if mode == 'JSON':
            json.loads(content,parse_constant=lambda v:(_ for _ in ()).throw(ValueError('JSON 不允许 NaN/Infinity')))
        try:
            payload = content.encode('gbk' if mode=='GBK' else 'utf-8')
        except UnicodeEncodeError as exc:
            raise ValueError('内容包含 GBK 无法编码的字符，可改用 UTF-8') from exc
        payload += {'None':b'','LF':b'\n','CRLF':b'\r\n'}[command['ending']]
    else:
        raise ValueError('未知命令格式')
    if not payload or not content.strip():
        raise ValueError('命令内容不能为空')
    if len(payload)>65507:
        raise ValueError('命令超过 IPv4 UDP 单包最大65507字节')
    return payload

def validate_target(ip, port):
    try:
        address = ipaddress.IPv4Address(ip.strip())
    except ipaddress.AddressValueError as exc:
        raise ValueError('请填写有效模块 IPv4 地址') from exc
    if address.is_unspecified or type(port) is not int or not 1 <= port <= 65535:
        raise ValueError('请填写有效目标地址和1～65535端口')
    return str(address),port
