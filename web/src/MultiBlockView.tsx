import Editor from "@monaco-editor/react";
import Plotly from "plotly.js-dist-min";
import { useCallback, useEffect, useRef, useState } from "react";

import {
  type BlockDefinition,
  type MultiBlockResult,
  planMultiBlock,
} from "./wasm/planner_wasm_loader";

function format(value: number, digits = 3) {
  return Number.isFinite(value) ? value.toFixed(digits) : "\u2014";
}

const PLOT_THEME = {
  paper_bgcolor: "#141414",
  plot_bgcolor: "#141414",
  font: { color: "#e6e6e6" },
};

const DEFAULT_BLOCKS_JSON = `[
  { millimeters: 5, maxEntrySpeed: 10, nominal: 200, aMax: 5000, jMax: 30000 },
  { millimeters: 5, maxEntrySpeed: 10, nominal: 200, aMax: 5000, jMax: 30000 },
  { millimeters: 5, maxEntrySpeed: 10, nominal: 200, aMax: 5000, jMax: 30000 },
]`;

export default function MultiBlockView() {
  const [blocksJson, setBlocksJson] = useState(DEFAULT_BLOCKS_JSON);
  const [result, setResult] = useState<MultiBlockResult | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [dt] = useState(0.0005);
  const plotRef = useRef<HTMLDivElement | null>(null);
  const [editorOpen, setEditorOpen] = useState(true);
  const [tableOpen, setTableOpen] = useState(true);

  const [plotHeight, setPlotHeight] = useState(600);

  useEffect(() => {
    const el = plotRef.current;
    if (!el) return;
    const ro = new ResizeObserver(([entry]) => {
      setPlotHeight(entry.contentRect.height);
    });
    ro.observe(el);
    return () => ro.disconnect();
  }, []);

  const runMultiBlock = useCallback(async () => {
    setError(null);
    try {
      // const blocks = JSON.parse(blocksJson) as BlockDefinition[];
      const blocks = Function(
        '"use strict"; return (' + blocksJson + ")"
      )() as BlockDefinition[];

      for (const b of blocks) {
        if (b.millimeters <= 0 || b.nominal <= 0 || b.aMax <= 0 || b.jMax <= 0)
          throw new Error("All block parameters must be positive");
        if (b.maxEntrySpeed < 0)
          throw new Error("maxEntrySpeed must be non-negative");
      }
      const res = await planMultiBlock(blocks, dt);
      setResult(res);
    } catch (err) {
      setError((err as Error).message);
      setResult(null);
    }
  }, [blocksJson, dt]);

  useEffect(() => {
    if (!plotRef.current || !result) return;

    const { times, position, velocity, acceleration, jerk, blockBoundaries } =
      result.trajectory;

    const shapes = blockBoundaries.map((t) => ({
      type: "line" as const,
      x0: t,
      x1: t,
      y0: 0,
      y1: 1,
      yref: "paper" as const,
      line: { color: "#555", width: 1, dash: "dot" as const },
    }));

    const layout = {
      grid: { rows: 4, columns: 1, pattern: "independent" },
      xaxis: { showticklabels: true },
      xaxis2: { matches: "x", showticklabels: true, title: "time (s)" },
      xaxis3: { matches: "x", showticklabels: true, title: "time (s)" },
      xaxis4: { matches: "x", showticklabels: true, title: "time (s)" },
      yaxis: { title: "position (mm)" },
      yaxis2: { title: "velocity (mm/s)", rangemode: "tozero" },
      yaxis3: { title: "acceleration (mm/s\u00b2)" },
      yaxis4: { title: "jerk (mm/s\u00b3)" },
      height: plotHeight,
      showlegend: true,
      ...PLOT_THEME,
      shapes,
    };

    const traces = [
      {
        x: times,
        y: position,
        mode: "lines",
        line: { width: 2, color: "deepskyblue" },
        name: "position",
        xaxis: "x",
        yaxis: "y",
      },
      {
        x: times,
        y: velocity,
        mode: "lines",
        line: { width: 2, color: "orange" },
        name: "velocity",
        xaxis: "x2",
        yaxis: "y2",
      },
      {
        x: times,
        y: acceleration,
        mode: "lines",
        line: { width: 2, color: "lime" },
        name: "acceleration",
        xaxis: "x3",
        yaxis: "y3",
      },
      {
        x: times,
        y: jerk,
        mode: "lines",
        line: { width: 2, color: "violet" },
        name: "jerk",
        xaxis: "x4",
        yaxis: "y4",
      },
    ];

    void Plotly.react(plotRef.current, traces, layout, {
      displaylogo: false,
      responsive: true,
    });
  }, [result, plotHeight]);

  useEffect(() => {
    runMultiBlock();
  }, [blocksJson, runMultiBlock]);
  return (
    <div className="flex min-h-0 flex-1 flex-col gap-6">
      <section className="rounded-xl border border-[#333] bg-[#1d1d1d]">
        <button
          type="button"
          className="flex w-full items-center gap-2 px-5 py-3 text-sm text-[#b8b8b8] hover:text-white"
          onClick={() => setEditorOpen((o) => !o)}
        >
          <span
            className="inline-block transition-transform"
            style={{
              transform: editorOpen ? "rotate(90deg)" : "rotate(0deg)",
            }}
          >
            &#9654;
          </span>
          Block definitions
        </button>
        {editorOpen && (
          <div className="px-5 pb-4">
            <div className="overflow-hidden rounded-md border border-[#333]">
              <Editor
                height="200px"
                defaultLanguage="javascript"
                value={blocksJson}
                onChange={(v) => setBlocksJson(v ?? "")}
                theme="vs-dark"
                options={{
                  minimap: { enabled: false },
                  scrollBeyondLastLine: false,
                  fontSize: 13,
                  lineNumbers: "off",
                  folding: false,
                  tabSize: 2,
                  automaticLayout: true,
                }}
              />
            </div>
            {error && <p className="mt-2 text-sm text-red-400">{error}</p>}
          </div>
        )}
      </section>

      {result && (
        <section className="rounded-xl border border-[#333] bg-[#1d1d1d]">
          <button
            type="button"
            className="flex w-full items-center gap-2 px-5 py-3 text-sm text-[#b8b8b8] hover:text-white"
            onClick={() => setTableOpen((o) => !o)}
          >
            <span
              className="inline-block transition-transform"
              style={{
                transform: tableOpen ? "rotate(90deg)" : "rotate(0deg)",
              }}
            >
              &#9654;
            </span>
            Blocks
            <span className="ml-auto text-xs text-[#666]">
              {result.blocks.length} blocks &middot;{" "}
              {format(result.totalDuration)} s &middot;{" "}
              {format(result.totalDistance, 2)} mm
            </span>
          </button>
          {tableOpen && (
            <div className="px-5 pb-4">
              <div className="overflow-x-auto max-h-40">
                <table className="w-full text-left text-sm">
                  <thead>
                    <tr className="border-b border-[#333] text-[#9aa3b2]">
                      <th className="px-2 py-1">#</th>
                      <th className="px-2 py-1">mm</th>
                      <th className="px-2 py-1">Nominal</th>
                      <th className="px-2 py-1">a_max</th>
                      <th className="px-2 py-1">j_max</th>
                      <th className="px-2 py-1">Entry V</th>
                      <th className="px-2 py-1">Exit V</th>
                    </tr>
                  </thead>
                  <tbody>
                    {result.blocks.map((b, i) => (
                      <tr key={i} className="border-b border-[#222]">
                        <td className="px-2 py-1 text-[#9aa3b2]">{i}</td>
                        <td className="px-2 py-1">
                          {format(b.input.millimeters, 2)}
                        </td>
                        <td className="px-2 py-1">
                          {format(b.input.nominal, 2)}
                        </td>
                        <td className="px-2 py-1">
                          {format(b.input.aMax, 2)}
                        </td>
                        <td className="px-2 py-1">
                          {format(b.input.jMax, 2)}
                        </td>
                        <td className="px-2 py-1 text-orange-300">
                          {format(b.entryV, 2)}
                        </td>
                        <td className="px-2 py-1 text-orange-300">
                          {format(b.exitV, 2)}
                        </td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
              <div className="mt-3 flex gap-6 text-sm text-[#b8b8b8]">
                <span>
                  Total duration: {format(result.totalDuration)} s
                </span>
                <span>
                  Total distance: {format(result.totalDistance, 2)} mm
                </span>
              </div>
            </div>
          )}
        </section>
      )}

      <section className="flex min-h-0 flex-1 flex-col rounded-xl border border-[#333] bg-[#1d1d1d] px-5 py-4">
        <div ref={plotRef} className="min-h-0 flex-1" />
      </section>
    </div>
  );
}
