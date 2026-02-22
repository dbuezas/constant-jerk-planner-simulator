declare module 'plotly.js-dist-min' {
  const Plotly: {
    newPlot: (...args: unknown[]) => Promise<void>
    react: (...args: unknown[]) => Promise<void>
  }
  export default Plotly
}
