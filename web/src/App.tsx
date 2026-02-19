import { useEffect, useMemo, useRef, useState } from 'react'
import Plotly from 'plotly.js-dist-min'
import { planAndSample, type TrajectorySample } from './wasm/planner_wasm_loader'

const DEFAULTS = {
  entryV: 0,
  exitV: 0,
  entryA: 0,
  exitA: 0,
  nominal: 100,
  maxEntry: 10,
  distance: 35,
  aMax: 500,
  jMax: 8000,
  dt: 0.0005,
}

const ANIM_DURATION = 500

type InputState = typeof DEFAULTS

function lerp(a: number, b: number, t: number) {
  return a + (b - a) * t
}

function lerpInputs(from: InputState, to: InputState, t: number): InputState {
  return {
    entryV: lerp(from.entryV, to.entryV, t),
    exitV: lerp(from.exitV, to.exitV, t),
    entryA: lerp(from.entryA, to.entryA, t),
    exitA: lerp(from.exitA, to.exitA, t),
    nominal: lerp(from.nominal, to.nominal, t),
    maxEntry: lerp(from.maxEntry, to.maxEntry, t),
    distance: lerp(from.distance, to.distance, t),
    aMax: lerp(from.aMax, to.aMax, t),
    jMax: lerp(from.jMax, to.jMax, t),
    dt: lerp(from.dt, to.dt, t),
  }
}

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
  const displayedInputsRef = useRef<InputState>(DEFAULTS)
  const animGenRef = useRef(0)

  function updateInput(key: keyof InputState, value: number) {
    setInputs(prev => {
      const next = { ...prev, [key]: value }
      if (key === 'entryV' || key === 'exitV') {
        next.nominal = Math.max(next.nominal, next.entryV, next.exitV)
      } else if (key === 'nominal') {
        next.entryV = Math.min(next.entryV, next.nominal)
        next.exitV = Math.min(next.exitV, next.nominal)
      } else if (key === 'entryA' || key === 'exitA') {
        next.aMax = Math.max(next.aMax, Math.abs(next.entryA), Math.abs(next.exitA))
      } else if (key === 'aMax') {
        next.entryA = Math.max(-next.aMax, Math.min(next.aMax, next.entryA))
        next.exitA = Math.max(-next.aMax, Math.min(next.aMax, next.exitA))
      }
      return next
    })
  }

  function handleShiftStep(e: React.KeyboardEvent<HTMLInputElement>, key: keyof InputState) {
    if (!e.shiftKey || (e.key !== 'ArrowUp' && e.key !== 'ArrowDown')) return
    e.preventDefault()
    const el = e.currentTarget
    const step = Number(el.step) * 10
    const current = Number(el.value)
    const delta = e.key === 'ArrowUp' ? step : -step
    const clamped = Math.min(Number(el.max||Number.POSITIVE_INFINITY), Math.max(Number(el.min||Number.NEGATIVE_INFINITY), current + delta))
    updateInput(key, clamped)
  }

  const [plotHeight, setPlotHeight] = useState(600)

  useEffect(() => {
    const el = plotRef.current
    if (!el) return
    const ro = new ResizeObserver(([entry]) => {
      setPlotHeight(entry.contentRect.height)
    })
    ro.observe(el)
    return () => ro.disconnect()
  }, [])

  const vmax = useMemo(() => {
    if (!planState.sample || planState.sample.velocity.length === 0) return 0
    return Math.max(...planState.sample.velocity)
  }, [planState.sample])

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
      yaxis2: { title: 'velocity (mm/s)', rangemode: 'tozero' },
      yaxis3: { title: 'acceleration (mm/s²)' },
      yaxis4: { title: 'jerk (mm/s³)' },
      height: plotHeight,
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
  }, [planState.sample, plotHeight])

  async function runPlan(planInputs: InputState) {
    setPlanState((prev) => ({ ...prev, error: null }))
    try {
      const sample = await planAndSample(
        {
          entryV: planInputs.entryV,
          entryA: planInputs.entryA,
          exitV: planInputs.exitV,
          exitA: planInputs.exitA,
          aMax: planInputs.aMax,
          jMax: planInputs.jMax,
          distance: planInputs.distance,
          nominal: planInputs.nominal,
          maxEntry: planInputs.maxEntry,
        },
        planInputs.dt
      )
      const ok = sample.duration > 0
      setPlanState({ sample, ok, error: ok ? null : 'Planner returned zero duration.' })
    } catch (err) {
      setPlanState({ sample: null, ok: false, error: (err as Error).message })
    }
  }

  useEffect(() => {
    const gen = ++animGenRef.current
    const from = { ...displayedInputsRef.current }
    const to = inputs
    const startTime = performance.now()

    function tick() {
      if (animGenRef.current !== gen) return
      const elapsed = performance.now() - startTime
      const t = Math.min(elapsed / ANIM_DURATION, 1)
      const interpolated = lerpInputs(from, to, t)
      displayedInputsRef.current = interpolated

      void runPlan(interpolated).then(() => {
        if (t < 1 && animGenRef.current === gen) {
          requestAnimationFrame(tick)
        }
      })
    }

    requestAnimationFrame(tick)
  }, [inputs])
if (planState.sample?.status!=="OK") console.log({planState, inputs})
  return (
    <div className="flex min-h-0 flex-1 flex-col gap-6">
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
              // max={200}
              step={1}
              onChange={(e) => updateInput('entryV', Number(e.target.value))}
              onKeyDown={(e) => handleShiftStep(e, 'entryV')}
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
              // max={200}
              step={1}
              onChange={(e) => updateInput('exitV', Number(e.target.value))}
              onKeyDown={(e) => handleShiftStep(e, 'exitV')}
            />
          </div>
          <div className="grid gap-1.5 text-left">
            <label className="text-sm text-[#b8b8b8]" htmlFor="entryA">Entry acceleration (mm/s²)</label>
            <input
              className="rounded-md border border-[#333] bg-[#111] px-2 py-2 text-white"
              id="entryA"
              type="number"
              value={inputs.entryA}
              // min={-2000}
              // max={2000}
              step={10}
              onChange={(e) => updateInput('entryA', Number(e.target.value))}
              onKeyDown={(e) => handleShiftStep(e, 'entryA')}
            />
          </div>
          <div className="grid gap-1.5 text-left">
            <label className="text-sm text-[#b8b8b8]" htmlFor="exitA">Exit acceleration (mm/s²)</label>
            <input
              className="rounded-md border border-[#333] bg-[#111] px-2 py-2 text-white"
              id="exitA"
              type="number"
              value={inputs.exitA}
              // min={-2000}
              // max={2000}
              step={10}
              onChange={(e) => updateInput('exitA', Number(e.target.value))}
              onKeyDown={(e) => handleShiftStep(e, 'exitA')}
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
              // max={200}
              step={1}
              onChange={(e) => updateInput('nominal', Number(e.target.value))}
              onKeyDown={(e) => handleShiftStep(e, 'nominal')}
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
              // max={200}
              step={1}
              onChange={(e) => updateInput('maxEntry', Number(e.target.value))}
              onKeyDown={(e) => handleShiftStep(e, 'maxEntry')}
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
              // max={200}
              step={1}
              onChange={(e) => updateInput('distance', Number(e.target.value))}
              onKeyDown={(e) => handleShiftStep(e, 'distance')}
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
              // max={20000}
              step={10}
              onChange={(e) => updateInput('aMax', Number(e.target.value))}
              onKeyDown={(e) => handleShiftStep(e, 'aMax')}
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
              // max={20000}
              step={100}
              onChange={(e) => updateInput('jMax', Number(e.target.value))}
              onKeyDown={(e) => handleShiftStep(e, 'jMax')}
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
              onChange={(e) => updateInput('dt', Number(e.target.value))}
              onKeyDown={(e) => handleShiftStep(e, 'dt')}
            />
          </div>
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
            <p className="text-lg">{planState.sample?.status || 'Not planned'}</p>
          </div>
          <div className="rounded-lg border border-[#2a2a2a] bg-[#121212] p-3">
            <h4 className="mb-2 text-sm text-[#9aa3b2]">Plan time</h4>
            <p className="text-lg">{format(planState.sample?.planTimeUs ?? 0, 3)} &micro;s</p>
          </div>
        </div>
        
        <div className="mt-4 flex items-center gap-3">
          {planState.error ? <span className="text-sm text-[#9aa3b2]">{planState.error}</span> : null}
        </div>
        
      </section>

      <section className="flex min-h-0 flex-1 flex-col rounded-xl border border-[#333] bg-[#1d1d1d] px-5 py-4">
        <div ref={plotRef} className="min-h-0 flex-1" />
      </section>
    </div>
  )
}

export default App
