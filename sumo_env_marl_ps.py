import gymnasium as gym
import numpy as np
import traci
import xml.etree.ElementTree as ET
import os


class SumoMARLEnvPS:
    """
    Multi-Agent Traffic Signal Control Environment with Parameter Sharing.

    Key differences from SumoMARLEnv (independent learners):
    - Each agent's observation is augmented with a normalised intersection
      index (agent_index / num_agents) appended as the final element.
    - This allows a SINGLE shared PPO model to distinguish which
      intersection is acting while keeping observation space size
      constant regardless of network size (local_obs_size + 1).
    - Normalised index scales naturally to 4 and 6 intersection networks.

    Observation per agent: [local_queue_lanes..., phase_split, norm_id]
    All padded to uniform size: max_local_obs + 1

    Handles mixed phase structures per junction (added for 4int/6int
    reuse): standard 4-phase junctions use the simple two-green formula;
    any junction with more phases (e.g. the 6-phase protected-left
    junction cluster_54994556_7161921429) uses the generalized
    multi-stage formula with a per-junction split ceiling so the final
    green stage never drops below yellow_time. This branch never
    triggers on the 2-intersection network (no junction there has more
    than 4 phases), so 2int behavior is unchanged.
    """

    def __init__(self, config):

        # ── Network config ─────────────────────────────────────────────
        self.sumocfg   = config["sumocfg"]
        self.use_gui   = config.get("use_gui", False)
        self.agent_ids = config["agent_ids"]
        self.tl_map    = config["tl_map"]
        self.lane_map  = config["lane_map"]
        self.num_agents= len(self.agent_ids)

        # ── Normalised intersection index per agent ─────────────────────
        # agent_index / num_agents — unique, fixed size, scales to any N
        self.agent_index = {
            agent: i / self.num_agents
            for i, agent in enumerate(self.agent_ids)
        }

        # ── Observation size ───────────────────────────────────────────
        # local obs = lane queues + phase split
        # pad all to max local obs size, then append norm ID
        self.local_obs_sizes = {
            agent: len(self.lane_map[agent]) + 1
            for agent in self.agent_ids
        }
        self.max_local_obs = max(self.local_obs_sizes.values())
        self.obs_size      = self.max_local_obs + 1  # +1 for norm ID

        # ── Phase split parameters ─────────────────────────────────────
        self.delta_s      = config.get("delta_s", 5)
        self.s_lb         = config.get("s_lb", 15)
        self.s_ub         = config.get("s_ub", 63)
        self.cycle_length = config.get("cycle_length", 90)
        self.yellow_time  = config.get("yellow_time", 6)
        self.allred_time  = config.get("allred_time", 0)
        self.init_split   = config.get("init_split", 39)

        # ── Optional: real all-red phases (default OFF = old behaviour) ─
        # When True, each junction (must have 4 net phases: gA,yA,gB,yB) is
        # rebuilt as gA, yA, all-red, gB, yB, all-red. Yellow comes from
        # yellow_time, all-red from allred_time (both from config, not net).
        self.allred_phases = config.get("allred_phases", False)
        # When True, step() also accumulates arrivals/teleports over EVERY
        # sim second of the cycle (last_arrived/last_teleports only hold the
        # final second, which under-counts). Default OFF = old behaviour.
        self.track_step_metrics = config.get("track_step_metrics", False)
        # Debug aids (defaults = old behaviour)
        self.error_log = config.get("error_log", None)       # SUMO stderr file
        self.allred_set_phase = config.get("allred_set_phase", True)
        self.allred_program_id = config.get("allred_program_id", "tod6")

        # ── Reward parameters ──────────────────────────────────────────
        self.q_lc = config.get("q_lc", 5)
        self.q_hc = config.get("q_hc", 15)
        self.w_l  = config.get("w_l", 1.0)
        self.w_cp = config.get("w_cp", 3.0)

        # All monitored lanes across all agents for global reward
        self.all_lanes = [
            lane
            for agent in self.agent_ids
            for lane in self.lane_map[agent]
        ]

        # ── Episode parameters ─────────────────────────────────────────
        self.sim_steps     = config.get("sim_steps", 3600)
        self.step_length   = config.get("step_length", 90)
        self.current_step  = 0
        self.episode_count = 0

        # ── Internal state ─────────────────────────────────────────────
        self.phase_splits = {
            agent: self.init_split for agent in self.agent_ids
        }
        self.last_arrived     = 0
        self.last_teleports   = 0
        self.mean_travel_time = 0.0
        self.mean_delay       = 0.0
        self.step_arrived     = 0      # whole-cycle totals (track_step_metrics)
        self.step_teleports   = 0
        self.tripinfo_records = []     # per-vehicle dicts from last episode
        self._base_states     = {}     # agent -> (gA, yA, gB, yB) from net
        self._base_meta       = {}     # agent -> (programID, type) from net
        self._active_prog     = {}     # agent -> program id now running
        self._created_progs   = {a: set() for a in self.agent_ids}

        # ── Gymnasium spaces ───────────────────────────────────────────
        # Same for all agents — shared model requires uniform obs size
        self.observation_space = gym.spaces.Box(
            low=0.0, high=1.0,
            shape=(self.obs_size,),
            dtype=np.float32
        )
        self.action_space = gym.spaces.Discrete(3)

        print(f"MARL-PS Environment initialised:")
        print(f"  Agents:            {self.agent_ids}")
        print(f"  Max local obs:     {self.max_local_obs}")
        print(f"  Obs size (+ ID):   {self.obs_size}")
        print(f"  Action space:      Discrete(3) per agent")
        print(f"  Normalised IDs:    {self.agent_index}")
        print(f"  All lanes:         {len(self.all_lanes)}")

    # ── Get observation for one agent (with normalised ID) ─────────────
    def _get_agent_obs(self, agent):
        try:
            lanes  = self.lane_map[agent]
            queues = [traci.lane.getLastStepHaltingNumber(l) for l in lanes]
            split  = self.phase_splits[agent]

            q_max  = 50.0
            norm_q = [min(q / q_max, 1.0) for q in queues]
            norm_s = [(split - self.s_lb) / (self.s_ub - self.s_lb)]

            # Local obs padded to max_local_obs
            local_obs = np.array(norm_q + norm_s, dtype=np.float32)
            if len(local_obs) < self.max_local_obs:
                local_obs = np.pad(local_obs,
                                   (0, self.max_local_obs - len(local_obs)))

            # Append normalised intersection ID
            norm_id = np.array([self.agent_index[agent]], dtype=np.float32)
            return np.concatenate([local_obs, norm_id])

        except Exception:
            return np.zeros(self.obs_size, dtype=np.float32)

    # ── Get observations for all agents ────────────────────────────────
    def _get_observations(self):
        return {agent: self._get_agent_obs(agent)
                for agent in self.agent_ids}

    # ── Global reward — network-wide queue penalty ─────────────────────
    def _compute_reward(self):
        try:
            total = 0.0
            for lane in self.all_lanes:
                q = traci.lane.getLastStepHaltingNumber(lane)
                if q <= self.q_lc:
                    total += 0.0
                elif q <= self.q_hc:
                    total += -(self.w_l * q)
                else:
                    total += -(self.w_cp * self.w_l * q)
            teleports = traci.simulation.getStartingTeleportNumber()
            total    += -(5.0 * teleports)
            return total
        except Exception:
            return 0.0

    # ── All-red phase support (only used when allred_phases=True) ──────
    def _intergreen(self):
        return self.yellow_time + self.allred_time

    def _split_ub(self):
        """Largest phase-A green that still leaves phase B >= s_lb."""
        return min(self.s_ub,
                   self.cycle_length - 2 * self._intergreen() - self.s_lb)

    def _install_allred_logic(self, agent, split, first=False):
        """
        Build gA, yA, [allred], gB, yB, [allred] for one junction using the
        green/yellow states cached from the net at reset. Phase B green is
        whatever is left of the cycle, so the cycle is always exactly
        cycle_length.
        """
        tl_id = self.tl_map[agent]
        gA, yA, gB, yB = self._base_states[agent]
        red = "r" * len(gA)
        b_green = (self.cycle_length - split - 2 * self._intergreen())
        P = traci.trafficlight.Phase

        phases = [P(split, gA), P(self.yellow_time, yA)]
        if self.allred_time > 0:
            phases.append(P(self.allred_time, red))
        phases += [P(b_green, gB), P(self.yellow_time, yB)]
        if self.allred_time > 0:
            phases.append(P(self.allred_time, red))

        # SUMO crashes if a running program is replaced by one with a
        # different phase count, and replacing a program that has not run yet
        # is also suspect. So: one program per split value (id "<base>_<split>"),
        # created once, never modified, switched to with setProgram when the
        # split changes. Unchanged split = no TraCI calls at all.
        _, ptype = self._base_meta[agent]
        pid = f"{self.allred_program_id}_{split:g}"
        if self._active_prog.get(agent) == pid:
            return
        if pid not in self._created_progs[agent]:
            traci.trafficlight.setProgramLogic(
                tl_id, traci.trafficlight.Logic(pid, ptype, 0, phases))
            self._created_progs[agent].add(pid)
        traci.trafficlight.setProgram(tl_id, pid)
        self._active_prog[agent] = pid
        if self.allred_set_phase:
            # we are at a cycle boundary, so restart at phase 0 with a full
            # phase-0 timer (a revisited program may hold a stale phase)
            traci.trafficlight.setPhase(tl_id, 0)

    def set_splits(self, splits):
        """
        Set absolute phase-A splits (e.g. from a time-of-day schedule).
        Takes effect when the next step() applies its (no-op) actions.
        Clipped to [s_lb, upper bound] so SARTSM minimums always hold.
        """
        ub = self._split_ub() if self.allred_phases else self.s_ub
        for agent, s in splits.items():
            self.phase_splits[agent] = float(np.clip(s, self.s_lb, ub))

    # ── Apply action for one agent ─────────────────────────────────────
    def _apply_action(self, agent, action_idx):
        tl_id     = self.tl_map[agent]
        delta_map = {0: -self.delta_s, 1: 0, 2: self.delta_s}
        delta     = delta_map[int(action_idx)]

        if self.allred_phases:
            new_split = float(np.clip(
                self.phase_splits[agent] + delta,
                self.s_lb, self._split_ub()))
            self.phase_splits[agent] = new_split
            self._install_allred_logic(agent, new_split)
            return

        logic  = traci.trafficlight.getAllProgramLogics(tl_id)[0]
        phases = logic.phases

        if len(phases) == 4:
            effective_ub = self.s_ub
        else:
            # Multi-stage junction (e.g. 6-phase protected-left): derive
            # fixed overhead from the junction's own phases and cap the
            # split so the final green stage keeps >= one yellow_time.
            middle_greens = sum(phases[i].duration
                                for i in range(2, len(phases) - 2, 2))
            yellows       = sum(phases[i].duration
                                for i in range(1, len(phases), 2))
            max_split_for_junction = (self.cycle_length
                                       - middle_greens
                                       - yellows
                                       - self.allred_time
                                       - self.yellow_time)
            effective_ub = min(self.s_ub, max_split_for_junction)

        new_split = float(np.clip(
            self.phase_splits[agent] + delta,
            self.s_lb, effective_ub
        ))
        self.phase_splits[agent] = new_split

        if len(phases) == 4:
            ew_green = (self.cycle_length
                        - new_split
                        - 2 * self.yellow_time
                        - self.allred_time)
            new_phases = [
                traci.trafficlight.Phase(new_split, phases[0].state),
                phases[1],
                traci.trafficlight.Phase(ew_green,  phases[2].state),
                phases[3],
            ]
        else:
            middle_greens = sum(phases[i].duration
                                for i in range(2, len(phases) - 2, 2))
            yellows       = sum(phases[i].duration
                                for i in range(1, len(phases), 2))
            last_green    = (self.cycle_length - new_split
                             - middle_greens - yellows - self.allred_time)

            new_phases = [phases[i] for i in range(len(phases))]
            new_phases[0] = traci.trafficlight.Phase(
                new_split, phases[0].state)
            new_phases[len(phases) - 2] = traci.trafficlight.Phase(
                last_green, phases[len(phases) - 2].state)

        new_logic = traci.trafficlight.Logic(
            logic.programID, logic.type,
            logic.currentPhaseIndex, new_phases
        )
        traci.trafficlight.setProgramLogic(tl_id, new_logic)

    # ── Parse tripinfo ──────────────────────────────────────────────────
    def _parse_tripinfo(self):
        try:
            path = f"tripinfo_marl_ps_ep{self.episode_count}.xml"
            if not os.path.exists(path):
                return 0.0, 0.0
            tree = ET.parse(path)
            root = tree.getroot()
            tt, dl = [], []
            self.tripinfo_records = []
            for trip in root.findall('tripinfo'):
                d = trip.get('duration')
                t = trip.get('timeLoss')
                if d: tt.append(float(d))
                if t: dl.append(float(t))
                if d and t and trip.get('depart'):
                    self.tripinfo_records.append({
                        "id"       : trip.get('id'),
                        "depart"   : float(trip.get('depart')),
                        "duration" : float(d),
                        "timeLoss" : float(t),
                        "departDelay": float(trip.get('departDelay') or 0.0),
                    })
            mean_tt  = float(np.mean(tt)) if tt else 0.0
            mean_del = float(np.mean(dl)) if dl else 0.0
            os.remove(path)
            return mean_tt, mean_del
        except Exception:
            return 0.0, 0.0

    # ── Reset ──────────────────────────────────────────────────────────
    def reset(self, seed=None, route_file=None):
        try:
            traci.close()
        except Exception:
            pass

        self.episode_count += 1
        binary = "sumo-gui" if self.use_gui else "sumo"
        cmd = [
            binary, "-c", self.sumocfg,
            "--no-step-log", "true",
            "--tripinfo-output",
            f"tripinfo_marl_ps_ep{self.episode_count}.xml"
        ]
        if seed is not None:
            cmd += ["--seed", str(seed)]
        if route_file is not None:
            cmd += ["--route-files", route_file]   # overrides sumocfg's file
        if self.error_log:
            cmd += ["--error-log", self.error_log]
        traci.start(cmd)

        self.current_step     = 0
        self.last_arrived     = 0
        self.last_teleports   = 0
        self.step_arrived     = 0
        self.step_teleports   = 0
        self.tripinfo_records = []
        self.mean_travel_time = 0.0
        self.mean_delay       = 0.0
        self.phase_splits     = {
            agent: self.init_split for agent in self.agent_ids
        }

        self._active_prog   = {}
        self._created_progs = {a: set() for a in self.agent_ids}
        if self.allred_phases:
            # Cache the net's own green/yellow states, then install the
            # 6-phase (all-red) logic at init_split.
            for agent in self.agent_ids:
                logic = traci.trafficlight.getAllProgramLogics(
                    self.tl_map[agent])[0]
                ph = logic.phases
                if len(ph) != 4:
                    raise ValueError(
                        f"allred_phases needs a 4-phase net logic for "
                        f"{agent}, found {len(ph)} phases")
                self._base_states[agent] = (ph[0].state, ph[1].state,
                                            ph[2].state, ph[3].state)
                self._base_meta[agent] = (logic.programID, logic.type)
                self._install_allred_logic(agent, self.init_split,
                                           first=True)

        return self._get_observations()

    # ── Step ───────────────────────────────────────────────────────────
    def step(self, actions):
        """
        actions: dict {agent_id: action_index}
        All agents act simultaneously before SUMO advances.
        """
        for agent, action_idx in actions.items():
            self._apply_action(agent, action_idx)

        self.step_arrived   = 0
        self.step_teleports = 0
        for _ in range(self.step_length):
            if traci.simulation.getMinExpectedNumber() == 0:
                self.current_step = self.sim_steps
                break
            traci.simulationStep()
            self.current_step += 1
            if self.track_step_metrics:
                self.step_arrived   += traci.simulation.getArrivedNumber()
                self.step_teleports += \
                    traci.simulation.getStartingTeleportNumber()

        reward     = self._compute_reward()
        terminated = self.current_step >= self.sim_steps

        try:
            self.last_arrived   = traci.simulation.getArrivedNumber()
            self.last_teleports = traci.simulation.getStartingTeleportNumber()
        except Exception:
            pass

        observations = self._get_observations()
        rewards      = {agent: reward for agent in self.agent_ids}
        terminateds  = {agent: terminated for agent in self.agent_ids}

        if terminated:
            try:
                traci.close()
            except Exception:
                pass
            self.mean_travel_time, self.mean_delay = self._parse_tripinfo()

        return observations, rewards, terminateds

    # ── Close ──────────────────────────────────────────────────────────
    def close(self):
        try:
            traci.close()
        except Exception:
            pass