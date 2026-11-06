"""Live two-intersection SUMO with a deterministic policy and virtual serial.

This tests integration, never physical USB latency or the trained PPO model.
"""
import importlib
import json
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import pytest

from supervisor.tests.test_hil_timing_GPT import BoardDouble, Clock, read_rows


def test_live_sumo_fault_handback_and_matching_timing_receipts(tmp_path, monkeypatch):
    if not shutil.which('sumo'):
        pytest.skip('SUMO not installed')
    project = Path(__file__).resolve().parents[2]
    network = project/'2int_files/osm.net.xml'
    if not network.is_file() or not (project/'sumo_env_marl_ps.py').is_file():
        pytest.skip('Full two-intersection project required')
    traci = pytest.importorskip('traci')
    pytest.importorskip('gymnasium')
    import tod_eval_common as ec
    import tod_plan as tp
    from sumo_env_marl_ps import SumoMARLEnvPS
    from supervisor.config import load_config
    from supervisor.conflicts import load_fixture
    from supervisor.hil_display_GPT import PhaseDisplay
    from supervisor.hil_serial_GPT import SerialDisplay
    from supervisor.observation_adapter_GPT import StrictPolicy
    from supervisor.supervised_integrated_GPT import IntegratedSupervisedController
    from run_hil_fault_test_GPT import OneShotFault, verify
    sys.modules.pop('supervisor.integrated_runner_GPT', None)
    runner = importlib.import_module('supervisor.integrated_runner_GPT')

    route = tmp_path/'routes.xml'
    source = project/'2int_files/routes.xml'
    if not source.is_file():
        pytest.skip('Route mix required')
    tp.generate_route_file(str(route),tp.load_route_mix(str(source)),42,ec.DEFAULT_SCALE*tp.ORIGINAL_RATE_VPH)
    xml = ET.Element('configuration'); inputs = ET.SubElement(xml,'input')
    ET.SubElement(inputs,'net-file',value=str(network))
    ET.SubElement(inputs,'route-files',value=str(route))
    scenario = tmp_path/'test.sumocfg'
    ET.ElementTree(xml).write(scenario)
    config = ec.make_config(); config.update(sumocfg=str(scenario),use_gui=False)
    cfg = load_config(); matrix = load_fixture(project / cfg.conflicts.fixture_path)
    env = SumoMARLEnvPS(config)
    class NoChangePolicy:
        def predict(self, obs, deterministic=True): return 1, None
    policy = StrictPolicy('marl',NoChangePolicy(),traci)
    injector = OneShotFault(cfg)
    def factory(inner,cfg,matrix,**kwargs):
        return IntegratedSupervisedController(inner,cfg,matrix,snapshot_taker=injector,**kwargs)
    clock = Clock(); board = BoardDouble(clock)
    monkeypatch.chdir(tmp_path)
    try:
        with SerialDisplay('VIRTUAL',log_path=tmp_path/'display_GPT.jsonl',serial_instance=board,clock=clock) as transport:
            display = PhaseDisplay(transport,speed=1,sleep=clock.sleep)
            summary = runner.run_episode(env,policy,cfg,matrix,42,route,tmp_path,
                slow_enabled=False,handback_enabled=True,max_cycles=10,
                controller_factory=factory,display=display)
        result = verify(tmp_path,summary,injector.injection)
        assert result['status'] == 'PASS', result
        rows = read_rows(tmp_path/'timing_GPT.jsonl')
        assert len(rows) == 10 and all(r['status']=='complete' for r in rows)
        assert [r['sim_time'] for r in rows] == list(range(0,900,90))
        assert all(r['first_step_output']['sim_time']==r['sim_time']+1 for r in rows)
        assert all(r['first_step_output']['ack']['sequence']==r['boundary_output']['ack']['sequence']+1 for r in rows)
        assert json.loads((tmp_path/'timing_summary_GPT.json').read_text())['measurement_status']=='COMPLETE'
    finally:
        env.close()
        sys.modules.pop('supervisor.integrated_runner_GPT', None)
