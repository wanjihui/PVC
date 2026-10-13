from dataclasses import asdict
from app import startup_rules
from packet_parser import DEFAULT_RULES, TRACK_RULES, FieldRule


def test_fresh_invalid_and_old_three_angle_settings_use_track():
    assert startup_rules({}) == TRACK_RULES
    assert startup_rules({'rules':[{'bad':'value'}]}) == TRACK_RULES
    assert startup_rules({'rules':[asdict(r) for r in DEFAULT_RULES]}) == TRACK_RULES


def test_explicit_new_version_choice_and_custom_rules_are_preserved():
    assert startup_rules({'rules_schema_version':2,'rules':[asdict(r) for r in DEFAULT_RULES]}) == DEFAULT_RULES
    custom = [FieldRule('mine','int32',12)]
    assert startup_rules({'rules':[asdict(r) for r in custom]}) == custom


def test_full_track_default_retains_control_and_speed_fields():
    names = {r.name for r in startup_rules({})}
    assert {'vis_err_px','omega_cmd_dps','vx_cmd_cms','vy_cmd_cms','speed_FR','target_FR','duty_FR'} <= names
