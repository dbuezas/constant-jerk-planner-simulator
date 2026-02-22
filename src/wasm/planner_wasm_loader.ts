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
  mergedCount: number;
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
  const mergedSize = mod.cwrap("cjp_merged_size", "number", []);
  const execReset = mod.cwrap("cjp_exec_reset", null, []);
  const execStep = mod.cwrap("cjp_exec_step", "number", ["number"]);
  const execTime = mod.cwrap("cjp_exec_time", "number", []);
  const execPosition = mod.cwrap("cjp_exec_position", "number", []);
  const execVelocity = mod.cwrap("cjp_exec_velocity", "number", []);
  const execAcceleration = mod.cwrap("cjp_exec_acceleration", "number", []);
  const execJerk = mod.cwrap("cjp_exec_jerk", "number", []);
  const execOrigBlock = mod.cwrap("cjp_exec_original_block", "number", []);
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

  const mergedCount = mergedSize() as number;

  // Read per-original-block data for the results table
  const resultBlocks: MultiBlockResult["blocks"] = [];
  const ptr = mallocFn(7 * 4);
  try {
    for (let i = 0; i < blocks.length; i++) {
      const gotBlock = getBlockData(i, ptr);
      if (gotBlock) {
        const bd = readBlock(mod, ptr);
        resultBlocks.push({
          input: blocks[i],
          entryV: bd!.entryV,
          exitV: bd!.exitV,
        });
      }
    }
  } finally {
    freeFn(ptr);
  }

  // Stream through the trajectory using exec API
  const allTimes: number[] = [];
  const allPos: number[] = [];
  const allVel: number[] = [];
  const allAcc: number[] = [];
  const allJrk: number[] = [];
  const blockBoundaries: number[] = [0];

  let lastOrigBlock = 0;

  execReset();
  while (execStep(dt)) {
    allTimes.push(execTime());
    allPos.push(execPosition());
    allVel.push(execVelocity());
    allAcc.push(execAcceleration());
    allJrk.push(execJerk());

    const origBlock = execOrigBlock() as number;
    if (origBlock !== lastOrigBlock) {
      blockBoundaries.push(execTime());
      lastOrigBlock = origBlock;
    }
  }

  const totalDuration = allTimes.length > 0 ? allTimes[allTimes.length - 1] : 0;
  const totalDistance = allPos.length > 0 ? allPos[allPos.length - 1] : 0;

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
    totalDuration,
    totalDistance,
    mergedCount,
  };
}
