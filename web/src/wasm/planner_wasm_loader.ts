type EmscriptenModule = {
  cwrap: (
    name: string,
    returnType: string | null,
    argTypes: string[]
  ) => (...args: number[]) => number;
  HEAPF32: Float32Array;
};

type WasmFactory = (options?: {
  locateFile?: (path: string) => string;
}) => Promise<EmscriptenModule>;

let modulePromise: Promise<EmscriptenModule> | null = null;

export async function loadPlannerWasm(): Promise<EmscriptenModule> {
  if (!modulePromise) {
    const factory = (await import("./planner_wasm.js")).default as WasmFactory;
    modulePromise = factory({
      locateFile: (path) => new URL(`./${path}`, import.meta.url).toString(),
    });
  }
  return modulePromise;
}

export type TrajectoryPlanInput = {
  entryV: number;
  exitV: number;
  aMax: number;
  jMax: number;
  distance: number;
  nominal: number;
  maxEntry: number;
};

export type BlockDefinition = {
  millimeters: number;
  maxEntrySpeed: number;
  nominal: number;
  aMax: number;
  jMax: number;
};

export type MultiBlockResult = {
  blocks: Array<{
    input: BlockDefinition;
    entryV: number;
    exitV: number;
  }>;
  trajectory: {
    times: number[];
    position: number[];
    velocity: number[];
    acceleration: number[];
    jerk: number[];
    blockBoundaries: number[];
  };
  totalDuration: number;
  totalDistance: number;
};

export type TrajectorySample = {
  times: number[];
  position: number[];
  velocity: number[];
  acceleration: number[];
  jerk: number[];
  duration: number;
  planTimeUs: number;
  status: string;
  block: {
    millimeters: number;
    maxEntrySpeed: number;
    nominal: number;
    aMax: number;
    jMax: number;
    entryV: number;
    exitV: number;
  } | null;
};

function readBlock(
  mod: EmscriptenModule,
  ptr: number
): TrajectorySample["block"] {
  const base = ptr / 4;
  const buf = mod.HEAPF32.subarray(base, base + 7);
  return {
    millimeters: buf[0],
    maxEntrySpeed: buf[1],
    nominal: buf[2],
    aMax: buf[3],
    jMax: buf[4],
    entryV: buf[5],
    exitV: buf[6],
  };
}

export async function planAndSample(
  input: TrajectoryPlanInput,
  dt: number
): Promise<TrajectorySample> {
  const mod = await loadPlannerWasm();

  const reset = mod.cwrap("cjp_traj_reset", null, []);
  const plan = mod.cwrap("cjp_traj_plan", "number", [
    "number",
    "number",
    "number",
    "number",
    "number",
    "number",
  ]);
  const duration = mod.cwrap("cjp_traj_duration", "number", []);
  const position = mod.cwrap("cjp_traj_position", "number", ["number"]);
  const velocity = mod.cwrap("cjp_traj_velocity", "number", ["number"]);
  const acceleration = mod.cwrap("cjp_traj_acceleration", "number", ["number"]);
  const jerk = mod.cwrap("cjp_traj_jerk", "number", ["number"]);
  const planTimeUs = mod.cwrap("cjp_traj_plan_time_us", "number", []);
  const trajStatus = mod.cwrap(
    "cjp_traj_status",
    "string",
    []
  ) as unknown as () => string;

  const block: TrajectorySample["block"] = null;

  const entryV = input.entryV;
  const exitV = input.exitV;
  const aMax = input.aMax;
  const jMax = input.jMax;
  const distance = input.distance;
  const nominal = input.nominal;

  reset();
  const ok = plan(entryV, exitV, aMax, jMax, distance, nominal);
  const ptime = planTimeUs();
  const pstatus = trajStatus();
  if (!ok) {
    return {
      times: [],
      position: [],
      velocity: [],
      acceleration: [],
      jerk: [],
      duration: 0,
      planTimeUs: ptime,
      status: pstatus,
      block,
    };
  }

  const total = duration();
  if (total <= 0) {
    return {
      times: [],
      position: [],
      velocity: [],
      acceleration: [],
      jerk: [],
      duration: 0,
      planTimeUs: ptime,
      status: pstatus,
      block,
    };
  }

  const times: number[] = [];
  const pos: number[] = [];
  const vel: number[] = [];
  const acc: number[] = [];
  const jrk: number[] = [];

  const maxSamples = 10000;
  const targetStep = total / maxSamples;
  const step = Math.max(dt, targetStep);

  let t = 0;
  while (t <= total) {
    times.push(t);
    pos.push(position(t));
    vel.push(velocity(t));
    acc.push(acceleration(t));
    jrk.push(jerk(t));
    t += step;
  }
  if (times[times.length - 1] < total) {
    times.push(total);
    pos.push(position(total));
    vel.push(velocity(total));
    acc.push(acceleration(total));
    jrk.push(jerk(total));
  }

  return {
    times,
    position: pos,
    velocity: vel,
    acceleration: acc,
    jerk: jrk,
    duration: total,
    planTimeUs: ptime,
    status: pstatus,
    block,
  };
}

export async function planMultiBlock(
  blocks: BlockDefinition[],
  dt: number
): Promise<MultiBlockResult> {
  const mod = await loadPlannerWasm();

  const reset = mod.cwrap("cjp_reset", null, []);
  const pushBlock = mod.cwrap("cjp_push_block", "number", [
    "number", "number", "number", "number", "number",
  ]);
  const recalculate = mod.cwrap("cjp_recalculate", "number", []);
  const getBlockData = mod.cwrap("cjp_get_block_data", "number", [
    "number", "number",
  ]);
  const planBlockTraj = mod.cwrap("cjp_plan_block_trajectory", "number", [
    "number",
  ]);
  const trajDuration = mod.cwrap("cjp_traj_duration", "number", []);
  const trajPosition = mod.cwrap("cjp_traj_position", "number", ["number"]);
  const trajVelocity = mod.cwrap("cjp_traj_velocity", "number", ["number"]);
  const trajAcceleration = mod.cwrap("cjp_traj_acceleration", "number", [
    "number",
  ]);
  const trajJerk = mod.cwrap("cjp_traj_jerk", "number", ["number"]);
  const mallocFn = mod.cwrap("malloc", "number", ["number"]);
  const freeFn = mod.cwrap("free", null, ["number"]);

  reset();
  for (const b of blocks) {
    const ok = pushBlock(
      b.millimeters, b.maxEntrySpeed, b.nominal, b.aMax, b.jMax
    );
    if (!ok) throw new Error("Failed to push block (buffer full or invalid args)");
  }

  const feasible = recalculate();
  if (!feasible) throw new Error("Multi-block plan is infeasible");

  const resultBlocks: MultiBlockResult["blocks"] = [];
  const allTimes: number[] = [];
  const allPos: number[] = [];
  const allVel: number[] = [];
  const allAcc: number[] = [];
  const allJrk: number[] = [];
  const blockBoundaries: number[] = [];

  let cumulativeTime = 0;
  let cumulativePos = 0;

  const ptr = mallocFn(7 * 4);

  try {
    for (let i = 0; i < blocks.length; i++) {
      // Read block data
      const gotBlock = getBlockData(i, ptr);
      if (gotBlock) {
        const bd = readBlock(mod, ptr);
        resultBlocks.push({
          input: blocks[i],
          entryV: bd!.entryV,
          exitV: bd!.exitV,
        });
      }

      // Plan trajectory for this block
      const ok = planBlockTraj(i);
      if (!ok) throw new Error(`Failed to plan trajectory for block ${i}`);

      const dur = trajDuration();
      blockBoundaries.push(cumulativeTime);

      // Sample this block's trajectory
      const maxSamples = Math.ceil(10000 / blocks.length);
      const targetStep = dur / maxSamples;
      const step = Math.max(dt, targetStep);

      let t = 0;
      while (t <= dur) {
        allTimes.push(cumulativeTime + t);
        allPos.push(cumulativePos + trajPosition(t));
        allVel.push(trajVelocity(t));
        allAcc.push(trajAcceleration(t));
        allJrk.push(trajJerk(t));
        t += step;
      }
      if (allTimes[allTimes.length - 1] < cumulativeTime + dur) {
        allTimes.push(cumulativeTime + dur);
        allPos.push(cumulativePos + trajPosition(dur));
        allVel.push(trajVelocity(dur));
        allAcc.push(trajAcceleration(dur));
        allJrk.push(trajJerk(dur));
      }

      cumulativeTime += dur;
      cumulativePos += trajPosition(dur);
    }
  } finally {
    freeFn(ptr);
  }

  return {
    blocks: resultBlocks,
    trajectory: {
      times: allTimes,
      position: allPos,
      velocity: allVel,
      acceleration: allAcc,
      jerk: allJrk,
      blockBoundaries,
    },
    totalDuration: cumulativeTime,
    totalDistance: cumulativePos,
  };
}
