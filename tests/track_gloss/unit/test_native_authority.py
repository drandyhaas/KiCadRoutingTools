from kicad_track_gloss.kicad.authority import native_authority


class _Net:
    pass


class _ParentItem:
    def __init__(self, item_type, locked=False, parent=None):
        self.item_type = item_type
        self.locked = locked
        self.parent = parent

    def Type(self):
        return self.item_type

    def IsLocked(self):
        return self.locked

    def GetParentGroup(self):
        return self.parent


class _Parent:
    def __init__(self, item):
        self.item = item

    def AsEdaItem(self):
        return self.item


class _Track:
    def __init__(self, *, locked=False, parent=None):
        self.locked = locked
        self.parent = parent
        self.net = _Net()

    def IsLocked(self):
        return self.locked

    def GetParentGroup(self):
        return self.parent

    def GetNet(self):
        return self.net


class _Pcbnew:
    PCB_GENERATOR_T = 8


class _Board:
    def __init__(self, coupled=False):
        self.coupled = coupled

    def DpCoupledNet(self, _net):
        return object() if self.coupled else None


def test_manual_geometry_has_no_inferred_authority():
    assert native_authority(_Pcbnew, _Board(), _Track()) is None


def test_kicad_native_authorities_are_protected():
    assert native_authority(
        _Pcbnew, _Board(), _Track(locked=True)) == "locked_item"
    locked_group = _Parent(_ParentItem(7, locked=True))
    assert native_authority(
        _Pcbnew, _Board(), _Track(parent=locked_group)) == "locked_group"
    generator = _Parent(_ParentItem(_Pcbnew.PCB_GENERATOR_T))
    assert native_authority(
        _Pcbnew, _Board(), _Track(parent=generator)) == "generated"
    assert native_authority(
        _Pcbnew, _Board(coupled=True), _Track()) == "diff_pair"


def test_netclass_tuning_metadata_is_not_item_authority():
    assert native_authority(_Pcbnew, _Board(), _Track()) is None
