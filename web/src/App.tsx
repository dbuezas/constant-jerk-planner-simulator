import { useEffect, useMemo, useRef, useState } from 'react'
import Plotly from 'plotly.js-dist-min'
import { planAndSample, type TrajectorySample } from './wasm/planner_wasm_loader'

const DEFAULTS = {
  entryV: 0,
  exitV: 0,
  nominal: 100,
  maxEntry: 10,
  distance: 35,
  aMax: 500,
  jMax: 8000,
  dt: 0.0005,
}

type InputState = typeof DEFAULTS

type PlanState = {
  sample: TrajectorySample | null
  ok: boolean
  error: string | null
}

function format(value: number, digits = 3) {
  return Number.isFinite(value) ? value.toFixed(digits) : '—'
}

function App() {
  const [inputs, setInputs] = useState<InputState>(DEFAULTS)
  const [planState, setPlanState] = useState<PlanState>({ sample: null, ok: false, error: null })
  const plotRef = useRef<HTMLDivElement | null>(null)

  const vmax = useMemo(() => {
    if (!planState.sample || planState.sample.velocity.length === 0) return 0
    return Math.max(...planState.sample.velocity)
  }, [planState.sample])

  const plannedBlock = planState.sample?.block

  useEffect(() => {
    if (!plotRef.current || !planState.sample) return

    const { times, position, velocity, acceleration, jerk, duration } = planState.sample
    const boundary = duration

    const layout = {
      grid: { rows: 4, columns: 1, pattern: 'independent' },
      xaxis: { showticklabels: true },
      xaxis2: { matches: 'x', showticklabels: true, title: 'time (s)' },
      xaxis3: { matches: 'x', showticklabels: true, title: 'time (s)' },
      xaxis4: { matches: 'x', showticklabels: true, title: 'time (s)' },
      yaxis: { title: 'position (mm)' },
      yaxis2: { title: 'velocity (mm/s)' },
      yaxis3: { title: 'acceleration (mm/s²)' },
      yaxis4: { title: 'jerk (mm/s³)' },
      height: 1200,
      showlegend: true,
      paper_bgcolor: '#141414',
      plot_bgcolor: '#141414',
      font: { color: '#e6e6e6' },
      shapes: [
        {
          type: 'line',
          x0: boundary,
          x1: boundary,
          y0: 0,
          y1: 1,
          yref: 'paper',
          line: { color: '#888', width: 1, dash: 'dot' },
        },
      ],
    }

    const traces = [
      {
        x: times,
        y: position,
        mode: 'lines',
        line: { width: 2, color: 'deepskyblue' },
        name: 'position',
        xaxis: 'x',
        yaxis: 'y',
      },
      {
        x: times,
        y: velocity,
        mode: 'lines',
        line: { width: 2, color: 'orange' },
        name: 'velocity',
        xaxis: 'x2',
        yaxis: 'y2',
      },
      {
        x: times,
        y: acceleration,
        mode: 'lines',
        line: { width: 2, color: 'lime' },
        name: 'acceleration',
        xaxis: 'x3',
        yaxis: 'y3',
      },
      {
        x: times,
        y: jerk,
        mode: 'lines',
        line: { width: 2, color: 'violet' },
        name: 'jerk',
        xaxis: 'x4',
        yaxis: 'y4',
      },
    ]

    void Plotly.react(plotRef.current, traces, layout, { displaylogo: false, responsive: true })
  }, [planState.sample])

  async function runPlan() {
    setPlanState((prev) => ({ ...prev, error: null }))
    try {
      const sample = await planAndSample(
        {
          entryV: inputs.entryV,
          entryA: 0,
          exitV: inputs.exitV,
          exitA: 0,
          aMax: inputs.aMax,
          jMax: inputs.jMax,
          distance: inputs.distance,
          nominal: inputs.nominal,
          maxEntry: inputs.maxEntry,
        },
        inputs.dt
      )
      const ok = sample.duration > 0
      setPlanState({ sample, ok, error: ok ? null : 'Planner returned zero duration.' })
    } catch (err) {
      setPlanState({ sample: null, ok: false, error: (err as Error).message })
    } 
  }

  useEffect(() => {
    void runPlan()
  }, [inputs])

  return (
    <div className="grid gap-6">
      <section className="rounded-xl border border-[#333] bg-[#1d1d1d] px-5 py-4">
        <div className="grid gap-4 [grid-template-columns:repeat(auto-fit,minmax(200px,1fr))]">
          <div className="grid gap-1.5 text-left">
            <label className="text-sm text-[#b8b8b8]" htmlFor="entryV">Entry velocity (mm/s)</label>
            <input
              className="rounded-md border border-[#333] bg-[#111] px-2 py-2 text-white"
              id="entryV"
              type="number"
              value={inputs.entryV}
              min={0}
              max={200}
              step={1}
              onChange={(event) => setInputs({ ...inputs, entryV: Number(event.target.value) })}
            />
          </div>
          <div className="grid gap-1.5 text-left">
            <label className="text-sm text-[#b8b8b8]" htmlFor="exitV">Exit velocity (mm/s)</label>
            <input
              className="rounded-md border border-[#333] bg-[#111] px-2 py-2 text-white"
              id="exitV"
              type="number"
              value={inputs.exitV}
              min={0}
              max={200}
              step={1}
              onChange={(event) => setInputs({ ...inputs, exitV: Number(event.target.value) })}
            />
          </div>
          <div className="grid gap-1.5 text-left">
            <label className="text-sm text-[#b8b8b8]" htmlFor="nominal">Nominal velocity (mm/s)</label>
            <input
              className="rounded-md border border-[#333] bg-[#111] px-2 py-2 text-white"
              id="nominal"
              type="number"
              value={inputs.nominal}
              min={1}
              max={200}
              step={1}
              onChange={(event) => setInputs({ ...inputs, nominal: Number(event.target.value) })}
            />
          </div>
          <div className="grid gap-1.5 text-left">
            <label className="text-sm text-[#b8b8b8]" htmlFor="maxEntry">Max entry speed (mm/s)</label>
            <input
              className="rounded-md border border-[#333] bg-[#111] px-2 py-2 text-white"
              id="maxEntry"
              type="number"
              value={inputs.maxEntry}
              min={0}
              max={200}
              step={1}
              onChange={(event) => setInputs({ ...inputs, maxEntry: Number(event.target.value) })}
            />
          </div>
          <div className="grid gap-1.5 text-left">
            <label className="text-sm text-[#b8b8b8]" htmlFor="distance">Distance (mm)</label>
            <input
              className="rounded-md border border-[#333] bg-[#111] px-2 py-2 text-white"
              id="distance"
              type="number"
              value={inputs.distance}
              min={0}
              max={200}
              step={1}
              onChange={(event) => setInputs({ ...inputs, distance: Number(event.target.value) })}
            />
          </div>
          <div className="grid gap-1.5 text-left">
            <label className="text-sm text-[#b8b8b8]" htmlFor="aMax">A max (mm/s²)</label>
            <input
              className="rounded-md border border-[#333] bg-[#111] px-2 py-2 text-white"
              id="aMax"
              type="number"
              value={inputs.aMax}
              min={0}
              max={2000}
              step={1}
              onChange={(event) => setInputs({ ...inputs, aMax: Number(event.target.value) })}
            />
          </div>
          <div className="grid gap-1.5 text-left">
            <label className="text-sm text-[#b8b8b8]" htmlFor="jMax">J max (mm/s³)</label>
            <input
              className="rounded-md border border-[#333] bg-[#111] px-2 py-2 text-white"
              id="jMax"
              type="number"
              value={inputs.jMax}
              min={0}
              max={20000}
              step={100}
              onChange={(event) => setInputs({ ...inputs, jMax: Number(event.target.value) })}
            />
          </div>
          <div className="grid gap-1.5 text-left">
            <label className="text-sm text-[#b8b8b8]" htmlFor="dt">Sample dt (s)</label>
            <input
              className="rounded-md border border-[#333] bg-[#111] px-2 py-2 text-white"
              id="dt"
              type="number"
              value={inputs.dt}
              min={0.0001}
              max={0.01}
              step={0.0001}
              onChange={(event) => setInputs({ ...inputs, dt: Number(event.target.value) })}
            />
          </div>
        </div>
        <div className="mt-4 flex items-center gap-3">
          {planState.error ? <span className="text-sm text-[#9aa3b2]">{planState.error}</span> : null}
        </div>
      </section>

      <section className="rounded-xl border border-[#333] bg-[#1d1d1d] px-5 py-4">
        <div className="grid gap-3 [grid-template-columns:repeat(auto-fit,minmax(220px,1fr))]">
          <div className="rounded-lg border border-[#2a2a2a] bg-[#121212] p-3">
            <h4 className="mb-2 text-sm text-[#9aa3b2]">Duration (s)</h4>
            <p className="text-lg">{format(planState.sample?.duration ?? 0)}</p>
          </div>
          <div className="rounded-lg border border-[#2a2a2a] bg-[#121212] p-3">
            <h4 className="mb-2 text-sm text-[#9aa3b2]">Distance (mm)</h4>
            <p className="text-lg">{format(planState.sample?.position.at(-1) ?? 0)}</p>
          </div>
          <div className="rounded-lg border border-[#2a2a2a] bg-[#121212] p-3">
            <h4 className="mb-2 text-sm text-[#9aa3b2]">Vmax (mm/s)</h4>
            <p className="text-lg">{format(vmax)}</p>
          </div>
          <div className="rounded-lg border border-[#2a2a2a] bg-[#121212] p-3">
            <h4 className="mb-2 text-sm text-[#9aa3b2]">Status</h4>
            <p className="text-lg">{planState.ok ? 'OK' : 'Not planned'}</p>
          </div>
          <div className="rounded-lg border border-[#2a2a2a] bg-[#121212] p-3">
            <h4 className="mb-2 text-sm text-[#9aa3b2]">Planned entry v</h4>
            <p className="text-lg">{format(plannedBlock?.entryV ?? 0)}</p>
          </div>
          <div className="rounded-lg border border-[#2a2a2a] bg-[#121212] p-3">
            <h4 className="mb-2 text-sm text-[#9aa3b2]">Planned exit v</h4>
            <p className="text-lg">{format(plannedBlock?.exitV ?? 0)}</p>
          </div>
          <div className="rounded-lg border border-[#2a2a2a] bg-[#121212] p-3">
            <h4 className="mb-2 text-sm text-[#9aa3b2]">Planned entry a</h4>
            <p className="text-lg">{format(plannedBlock?.entryA ?? 0)}</p>
          </div>
          <div className="rounded-lg border border-[#2a2a2a] bg-[#121212] p-3">
            <h4 className="mb-2 text-sm text-[#9aa3b2]">Planned exit a</h4>
            <p className="text-lg">{format(plannedBlock?.exitA ?? 0)}</p>
          </div>
        </div>
      </section>


      <section className="min-h-[600px] rounded-xl border border-[#333] bg-[#1d1d1d] px-5 py-4">
        <div ref={plotRef} />
      </section>
    </div>
  )
}

export default App
