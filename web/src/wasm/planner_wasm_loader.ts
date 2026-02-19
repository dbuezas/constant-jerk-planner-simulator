type EmscriptenModule = {
  cwrap: (name: string, returnType: string | null, argTypes: string[]) => (...args: number[]) => number
  HEAPF32: Float32Array
}

type WasmFactory = (options?: { locateFile?: (path: string) => string }) => Promise<EmscriptenModule>

let modulePromise: Promise<EmscriptenModule> | null = null

export async function loadPlannerWasm(): Promise<EmscriptenModule> {
  if (!modulePromise) {
    const factory = (await import('./planner_wasm.js')).default as WasmFactory
    modulePromise = factory({
      locateFile: (path) => new URL(`./${path}`, import.meta.url).toString(),
    })
  }
  return modulePromise
}

export type TrajectoryPlanInput = {
  entryV: number
  entryA: number
  exitV: number
  exitA: number
  aMax: number
  jMax: number
  distance: number
  nominal: number
  maxEntry: number
}

export type TrajectorySample = {
  times: number[]
  position: number[]
  velocity: number[]
  acceleration: number[]
  jerk: number[]
  duration: number
  planTimeUs: number
  block: {
    millimeters: number
    maxEntrySpeed: number
    nominal: number
    aMax: number
    jMax: number
    entryV: number
    entryA: number
    exitV: number
    exitA: number
  } | null
}

function readBlock(mod: EmscriptenModule, ptr: number): TrajectorySample['block'] {
  const base = ptr / 4
  const buf = mod.HEAPF32.subarray(base, base + 9)
  return {
    millimeters: buf[0],
    maxEntrySpeed: buf[1],
    nominal: buf[2],
    aMax: buf[3],
    jMax: buf[4],
    entryV: buf[5],
    entryA: buf[6],
    exitV: buf[7],
    exitA: buf[8],
  }
}

export async function planAndSample(input: TrajectoryPlanInput, dt: number): Promise<TrajectorySample> {
  const mod = await loadPlannerWasm()

  const planBlock = mod.cwrap('cjp_plan_single_block', 'number', ['number', 'number', 'number', 'number', 'number'])
  const getBlock = mod.cwrap('cjp_get_first_block', 'number', ['number'])
  const reset = mod.cwrap('cjp_traj_reset', null, [])
  const plan = mod.cwrap('cjp_traj_plan', 'number', [
    'number',
    'number',
    'number',
    'number',
    'number',
    'number',
    'number',
    'number',
  ])
  const duration = mod.cwrap('cjp_traj_duration', 'number', [])
  const position = mod.cwrap('cjp_traj_position', 'number', ['number'])
  const velocity = mod.cwrap('cjp_traj_velocity', 'number', ['number'])
  const acceleration = mod.cwrap('cjp_traj_acceleration', 'number', ['number'])
  const jerk = mod.cwrap('cjp_traj_jerk', 'number', ['number'])
  const planTimeUs = mod.cwrap('cjp_traj_plan_time_us', 'number', [])
  const malloc = mod.cwrap('malloc', 'number', ['number'])
  const free = mod.cwrap('free', null, ['number'])

  const ptr = malloc(9 * 4)
  let block: TrajectorySample['block'] = null
  try {
    const okBlock = planBlock(input.distance, input.maxEntry, input.nominal, input.aMax, input.jMax)
    if (okBlock) {
      const got = getBlock(ptr)
      if (got) block = readBlock(mod, ptr)
    }
  } finally {
    free(ptr)
  }

  const entryV = input.entryV
  const entryA = block?.entryA ?? input.entryA
  const exitV = input.exitV
  const exitA = block?.exitA ?? input.exitA
  const aMax = block?.aMax ?? input.aMax
  const jMax = block?.jMax ?? input.jMax
  const distance = block?.millimeters ?? input.distance
  const nominal = input.nominal

  reset()
  const ok = plan(entryV, entryA, exitV, exitA, aMax, jMax, distance, nominal)
  const ptime = planTimeUs()
  if (!ok) {
    return {
      times: [],
      position: [],
      velocity: [],
      acceleration: [],
      jerk: [],
      duration: 0,
      planTimeUs: ptime,
      block,
    }
  }

  const total = duration()
  if (total <= 0) {
    return {
      times: [],
      position: [],
      velocity: [],
      acceleration: [],
      jerk: [],
      duration: 0,
      planTimeUs: ptime,
      block,
    }
  }

  const times: number[] = []
  const pos: number[] = []
  const vel: number[] = []
  const acc: number[] = []
  const jrk: number[] = []

  const maxSamples = 10000
  const targetStep = total / maxSamples
  const step = Math.max(dt, targetStep)

  let t = 0
  while (t <= total) {
    times.push(t)
    pos.push(position(t))
    vel.push(velocity(t))
    acc.push(acceleration(t))
    jrk.push(jerk(t))
    t += step
  }
  if (times[times.length - 1] < total) {
    times.push(total)
    pos.push(position(total))
    vel.push(velocity(total))
    acc.push(acceleration(total))
    jrk.push(jerk(total))
  }

  return {
    times,
    position: pos,
    velocity: vel,
    acceleration: acc,
    jerk: jrk,
    duration: total,
    planTimeUs: ptime,
    block,
  }
}
