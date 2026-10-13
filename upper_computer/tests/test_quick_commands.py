import json
import struct
import pytest
from quick_commands import new_command,defaults,encode_command,validate_config,load_config,save_config,validate_target

@pytest.mark.parametrize('mode,text,ending,expected',[
    ('UTF-8','停车','LF','停车\n'.encode()),
    ('GBK','停车','CRLF','停车\r\n'.encode('gbk')),
    ('HEX','AA 55 00 FF','LF',b'\xaa\x55\x00\xff'),
    ('JSON','{"cmd":"STOP"}','None',b'{"cmd":"STOP"}'),
])
def test_exact_encoding_without_name_or_note(mode,text,ending,expected):
    value = new_command('名称不发送')
    value.update(format=mode,content=text,ending=ending,note='备注不发送')
    assert encode_command(value)==expected

@pytest.mark.parametrize('order,marker',[('little','<'),('big','>')])
def test_binary_fields(order,marker):
    value = new_command()
    value.update(format='Binary',byte_order=order,ending='CRLF',content=json.dumps([
        dict(type='uint16',value=0x1234),dict(type='int32',value=-25),dict(type='float32',value=.5)]))
    assert encode_command(value)==struct.pack(marker+'Hif',0x1234,-25,.5)

@pytest.mark.parametrize('mode,text',[
    ('UTF-8',''),('HEX','A'),('HEX','ZZ'),('JSON','not-json'),('JSON','{"v":NaN}'),
    ('Binary','{}'),('Binary','[{"type":"uint8","value":256}]'),
    ('Binary','[{"type":"int16","value":1.2}]'),('Binary','[{"type":"bad","value":1}]'),
    ('UTF-8','a'*65508),('GBK','🙂')],ids=['empty','odd-hex','bad-hex','bad-json','nonfinite-json',
    'binary-object','binary-overflow','binary-fractional','unknown-type','oversized','gbk-character'])
def test_invalid_payload_is_rejected(mode,text):
    value=new_command()
    value.update(format=mode,content=text)
    with pytest.raises(ValueError):
        encode_command(value)

def test_storage_round_trip_empty_list_unicode_and_duplicate_ids(tmp_path):
    value=defaults()
    value['commands'][0].update(name='调速',note='可编辑的备注',content='SPEED,20')
    value['commands'].append(dict(value['commands'][0]))
    normalized=validate_config(value)
    assert normalized['commands'][0]['id']!=normalized['commands'][1]['id']
    path=tmp_path/'list.json'
    save_config(path,normalized)
    assert load_config(path)==normalized
    assert not list(tmp_path.glob('*.tmp_*'))
    normalized['commands']=[]
    save_config(path,normalized)
    assert load_config(path)['commands']==[]

def test_bad_import_does_not_replace_saved_list(tmp_path):
    path=tmp_path/'valid.json'
    original=defaults()
    save_config(path,original)
    invalid=tmp_path/'bad.json'
    invalid.write_text('{"version":99}',encoding='utf-8')
    with pytest.raises(ValueError):
        load_config(invalid)
    assert load_config(path)==original

@pytest.mark.parametrize('ip,port',[('',6666),('0.0.0.0',6666),('300.1.1.1',6666),('127.0.0.1',0),('127.0.0.1',True)])
def test_bad_destination(ip,port):
    with pytest.raises(ValueError):
        validate_target(ip,port)
