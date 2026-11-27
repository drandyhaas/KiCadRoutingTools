"""Declared net roles: KRT_GROUND_NETS / KRT_POWER_NETS.

The name rules find rails shaped like GND / VCC / +3V3. A board whose rails are
named otherwise (ISO_GND, ISO_3V3 on an isolated design) declares them as
comma-separated, case-insensitive fnmatch patterns in two environment variables.
Unset, nothing changes; a declared pattern never needs to be wide enough to also
catch a signal such as SW_GND_SENSE.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "py_router"))

import net_queries as nq  # noqa: E402


def _env(monkeypatch, ground=None, power=None):
    for key, val in (("KRT_GROUND_NETS", ground), ("KRT_POWER_NETS", power)):
        if val is None:
            monkeypatch.delenv(key, raising=False)
        else:
            monkeypatch.setenv(key, val)


def test_unset_keeps_the_name_rules(monkeypatch):
    _env(monkeypatch)
    assert nq.is_ground_net_name("GND")
    assert nq.is_ground_net_name("/GND")
    assert not nq.is_ground_net_name("ISO_GND")
    assert nq.is_power_net_name("VCC")
    assert not nq.is_power_net_name("ISO_3V3")


def test_declared_ground_and_power(monkeypatch):
    _env(monkeypatch, ground="ISO_GND, shield_rtn", power="ISO_3V3,MOTOR_24V*")
    assert nq.is_ground_net_name("ISO_GND")
    assert nq.is_ground_net_name("/sheet/SHIELD_RTN")     # sheet path stripped, case-insensitive
    assert nq.is_power_net_name("ISO_3V3")
    assert nq.is_power_net_name("MOTOR_24V_FUSED")        # fnmatch pattern
    assert nq.is_power_net_name("ISO_GND")                # a declared ground is a rail too
    assert not nq.is_power_net_name("ISO_3V3_SENSE_x")    # no implicit prefix match
    assert not nq.is_ground_net_name("SW_GND_SENSE")      # a signal stays a signal
    assert not nq.is_power_net_name("SW_GND_SENSE")


def test_empty_and_blank_entries(monkeypatch):
    _env(monkeypatch, ground=" , ,", power="")
    assert not nq.is_ground_net_name("ISO_GND")
    assert not nq.is_power_net_name("ISO_3V3")
    assert nq.is_ground_net_name("GND")
