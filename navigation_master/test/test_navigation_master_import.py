import importlib


def test_navigation_master_importable():
    mod = importlib.import_module('navigation_master.navigation_master')
    assert hasattr(mod, 'NavigationMaster')
