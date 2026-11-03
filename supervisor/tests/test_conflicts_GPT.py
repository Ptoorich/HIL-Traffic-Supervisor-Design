import gzip

import pytest

from supervisor.conflicts import (ConflictMatrixError, load_fixture, parse_net,
                                  save_fixture)

NET = """<net>
 <tlLogic id="J" type="static" programID="0" offset="0">
  <phase duration="30" state="Grg"/><phase duration="3" state="yry"/>
 </tlLogic>
 <junction id="J" type="traffic_light">
  <request index="0" response="000" foes="{r0}"/>
  <request index="1" response="000" foes="000"/>
  <request index="2" response="000" foes="{r2}"/>
 </junction>
</net>"""


def write(tmp_path, r0="100", r2="001", gz=False):
    p = tmp_path / ("n.net.xml.gz" if gz else "n.net.xml")
    data = NET.format(r0=r0, r2=r2).encode()
    (gzip.open(p, "wb") if gz else open(p, "wb")).write(data) if False else None
    if gz:
        with gzip.open(p, "wb") as f:
            f.write(data)
    else:
        p.write_bytes(data)
    return p


def test_bit_order_is_reversed_rightmost_is_link_zero(tmp_path):
    # link 0 conflicts with link 2: request 0 foes "100" (char 0 = link 2), request 2 foes "001"
    m = parse_net(write(tmp_path), {"tl1": "J"})
    j = m.junctions["tl1"]
    assert j.n_links == 3 and j.foes == frozenset({(0, 2)})
    assert j.net_phase_states == ("Grg", "yry")


def test_gzipped_net_and_fixture_round_trip(tmp_path):
    m = parse_net(write(tmp_path, gz=True), {"tl1": "J"})
    save_fixture(m, tmp_path / "f" / "fx.json", source="n")
    assert load_fixture(tmp_path / "f" / "fx.json") == m


def test_asymmetric_matrix_is_conservative(tmp_path):
    j = parse_net(write(tmp_path, r0="100", r2="000"), {"tl1": "J"}).junctions["tl1"]
    assert j.foes == frozenset({(0, 2)})
    assert j.asymmetric_pairs == frozenset({(0, 2)})


def test_self_foe_is_rejected(tmp_path):
    with pytest.raises(ConflictMatrixError, match="own foe"):
        parse_net(write(tmp_path, r0="001"), {"tl1": "J"})


def test_unknown_junction_and_missing_fixture(tmp_path):
    with pytest.raises(ConflictMatrixError, match="not found"):
        parse_net(write(tmp_path), {"tl1": "NOPE"})
    with pytest.raises(ConflictMatrixError, match="dump_conflicts"):
        load_fixture(tmp_path / "missing.json")


def test_two_agents_on_one_tl_id_is_rejected(tmp_path):
    with pytest.raises(ConflictMatrixError, match="share"):
        parse_net(write(tmp_path), {"tl1": "J", "tl2": "J"})
