# Section 3.4.4 — Four measured traffic-state screenshots (SUMO GUI)

This **isolated demonstration** uses the existing two-intersection `2int_files/osm.net.xml.gz` and valid routes taken from your `2int_files/routes.xml`. It does **not** execute the trained ASTA/RL policies, the live supervisor, Arduino outputs or normal benchmark demand. It deliberately blocks both traffic lights with all-red after the initial free/light screenshots, so queues can build and **HEAVY** and **GRIDLOCK** can be illustrated quickly. The blockage is artificial; do not present it as a safe traffic-control policy or a naturally occurring network failure.

The four classifications are **exactly those for the selected junction** in `supervisor/checks/traffic_classifier_GPT.py` and `supervisor/supervisor_config_GPT.toml`:

| State | Measured classifier rule |
|---|---|
| FREE | No jammed monitored lanes; mean queue fill <= 0.10 |
| LIGHT | No jammed monitored lanes; mean queue fill > 0.10 |
| HEAVY | One or two jammed monitored lanes |
| GRIDLOCK | Three or more of five monitored lanes jammed |

For each lane, "jammed" means **all three**: mean speed <0.5 m/s; occupancy at least 90% of jam occupancy (based on 5 m vehicle length and empirical lane capacity); at least 90% of the lane's halting-vehicle capacity. The current supervisor assesses each junction independently; these labels are not automatic network-wide classifications.

## Run on your Windows computer

1. Copy `run_flow_state_gui_GPT.py` into your existing `HIL-Traffic-Supervisor-Design` **repository root** (next to `tod_eval_common.py`). There is no need to modify any original project file.
2. In VS Code **Command Prompt** terminal, run:

```cmd
.venv\Scripts\activate
python run_flow_state_gui_GPT.py
```

3. SUMO GUI opens and centers on both intersections. The script pauses automatically once each target classification is detected. At each pause, take a screenshot (or use the automatically requested PNG), then press **Enter in the terminal** to continue. Do not press GUI Play; Python drives simulation steps through TraCI.
4. Look for images and the accompanying measurements CSV inside `supervisor_logs\flow_states_gui_GPT\`.

Default detection junction is **TL2**, the western cluster, but both intersections are visible and both classification labels are logged. To demonstrate **TL1** instead, choose an unused output folder and run:

```cmd
python run_flow_state_gui_GPT.py --focus tl1 --output-dir supervisor_logs\flow_states_tl1_GPT
```

If classification milestones are not reached, the script will **not** falsely label screenshots. Try a longer controlled run:

```cmd
python run_flow_state_gui_GPT.py --max-steps 2500 --output-dir supervisor_logs\flow_states_gui_02_GPT
```

To avoid input prompts and save screenshots automatically:

```cmd
python run_flow_state_gui_GPT.py --no-pause --output-dir supervisor_logs\flow_states_gui_auto_GPT
```

If the terminal reports `No module named traci`, confirm your `.venv` is active and SUMO's Python tools are installed (your project already uses TraCI). If it cannot find `sumo-gui`, ensure that SUMO's `bin` directory is on PATH or `SUMO_HOME` is correctly set. Use only one SUMO/TraCI demo session at a time.

## Figure captions and evidence

Each PNG is named with its **measured state** and the selected TL (`tl2` by default). The CSV includes the simulation time, both TL labels, mean queue fill and count of jammed monitored lanes. Attach that log or selected state rows in an appendix if needed. Use a caption such as:

*Illustrative SUMO screenshots of the two-intersection network under deliberately varied synthetic demand. Labels correspond to the implemented Traffic Classifier at TL2; high-congestion scenes were induced using an experimental all-red hold, not produced by the operational ASTA supervisor.*

Do not claim this demo validates sensor-fault detection, controller traffic performance or real-world gridlock thresholds. It is a **visual explanation**, with live model-based labels for the chosen junction. GUI appearance can differ between SUMO releases and display configurations. This script was checked for Python syntax and route-template availability against the saved model, but **not run against a local SUMO GUI instance in this workspace**.
